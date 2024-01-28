/** *************************************************************************************
 * Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
 * Copyright (c) 2020-2021 Peng Cheng Laboratory
 *
 * XiangShan is licensed under Mulan PSL v2.
 * You can use this software according to the terms and conditions of the Mulan PSL v2.
 * You may obtain a copy of Mulan PSL v2 at:
 * http://license.coscl.org.cn/MulanPSL2
 *
 * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
 * EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
 * MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
 *
 * See the Mulan PSL v2 for more details.
 * *************************************************************************************
 */

package coupledL2

import chisel3._
import chisel3.util._
import coupledL2.MetaData._
import xs.utils.{ParallelLookUp, ParallelPriorityMux}
import xs.utils.tl.MemReqSource
import freechips.rocketchip.tilelink._
import freechips.rocketchip.tilelink.TLMessages._
import freechips.rocketchip.tilelink.TLPermissions._
import org.chipsalliance.cde.config.Parameters
import coupledL2.prefetch.PrefetchTrain

class MSHRTasks(implicit p: Parameters) extends L2Bundle {
  // outer
  val source_a = DecoupledIO(new SourceAReq) // To AcquireUnit  // TODO: no need to use decoupled handshake
  val source_b = DecoupledIO(new SourceBReq)
  val mainpipe = DecoupledIO(new TaskBundle) // To Mainpipe (SourceC or SourceD)
  // val prefetchTrain = prefetchOpt.map(_ => DecoupledIO(new PrefetchTrain)) // To prefetcher
}

class MSHRResps(implicit p: Parameters) extends L2Bundle {
  val sink_c = Flipped(ValidIO(new RespInfoBundle))
  val sink_d = Flipped(ValidIO(new RespInfoBundle))
  val sink_e = Flipped(ValidIO(new RespInfoBundle))
  // make sure that Acquire is sent after Release,
  // so resp from SourceC is needed to initiate Acquire
  val source_c = Flipped(ValidIO(new RespInfoBundle))
}

class MSHR(implicit p: Parameters) extends L2Module {
  val io = IO(new Bundle() {
    val id = Input(UInt(mshrBits.W))
    val status = ValidIO(new MSHRStatus)
    val msInfo = ValidIO(new MSHRInfo)
    val alloc = Flipped(ValidIO(new MSHRRequest))
    val tasks = new MSHRTasks()
    val resps = new MSHRResps()
    val nestedwb = Input(new NestedWriteback)
    val nestedwbData = Output(Bool())
    val bMergeTask = Flipped(ValidIO(new BMergeTask))
    val replResp = Flipped(ValidIO(new ReplacerResult))
  })

  // For A need replace and it merge B
  val alreadySendProbe = RegInit(false.B)
  val alreadySRefill   = RegInit(false.B)
  val AneedReplMergeB  = RegInit(false.B)

  val alreadyNestC     = RegInit(false.B)
  val nestCIsXtoN      = RegInit(false.B)

  val gotT = RegInit(false.B) // L3 might return T even though L2 wants B
  val gotDirty = RegInit(false.B)
  val gotGrantData = RegInit(false.B)
  val gotGrant = RegInit(false.B)
  val probeDirty = RegInit(false.B)
  val probeGotN = RegInit(false.B)

  val timer = RegInit(0.U(64.W)) // for performance analysis

  /* MSHR Allocation */
//  val req_valid = RegInit(false.B)
  val req       = RegInit(0.U.asTypeOf(new TaskBundle()))
  val dirResult = RegInit(0.U.asTypeOf(new DirResult()))
  val meta      = dirResult.meta
  val initState = Wire(new FSMState())
  initState.elements.foreach(_._2 := true.B)
//  val state     = RegInit(new FSMState(), initState)
  val req_valid_dups = RegInit(VecInit(Seq.fill(6)(false.B)))
  val state_dups = RegInit(VecInit(Seq.fill(4)(initState)))
  val req_set_dups = RegInit(VecInit(Seq.fill(7)(0.U.asTypeOf(req.set))))
  val dirResult_tag_dups = RegInit(VecInit(Seq.fill(5)(0.U.asTypeOf(dirResult.tag))))

  when(io.alloc.valid) {
//    req_valid := true.B
//    state     := io.alloc.bits.state
    dirResult := io.alloc.bits.dirResult
    req       := io.alloc.bits.task
    gotT        := false.B
    gotDirty    := false.B
    gotGrantData := false.B
    gotGrant := false.B
    probeDirty  := false.B
    probeGotN   := false.B
    timer       := 1.U
    alreadySendProbe := false.B
    alreadySRefill   := false.B
    AneedReplMergeB  := false.B
    alreadyNestC     := false.B
    nestCIsXtoN      := false.B

    req_valid_dups.foreach(_ := true.B)
    state_dups.foreach(_ := io.alloc.bits.state)
    req_set_dups.foreach(_ := io.alloc.bits.task.set)
    dirResult_tag_dups.foreach(_ := io.alloc.bits.dirResult.tag)

    if(cacheParams.enableAssert) { assert(PopCount(state_dups.map(_.asUInt.andR.asBool)).asUInt === 4.U, "all state must be ture when alloc new task") }
  }

  /* ======== Enchantment ======== */
  val meta_pft = meta.prefetch.getOrElse(false.B)
  val meta_no_client = !meta.clients.orR

  val req_needT = needT(req.opcode, req.param)
  val req_acquire = req.opcode === AcquireBlock && req.fromA || req.opcode === AcquirePerm // AcquireBlock and Probe share the same opcode
  val req_acquirePerm = req.opcode === AcquirePerm
  val req_get = req.opcode === Get
  val req_prefetch = req.opcode === Hint

  val promoteT_normal =  dirResult.hit && meta_no_client && meta.state === TIP
  val promoteT_L3     = !dirResult.hit && gotT
  val promoteT_alias  =  dirResult.hit && req.aliasTask.getOrElse(false.B) && meta.state === TRUNK
  // under above circumstances, we grant T to L1 even if it wants B
  val req_promoteT = (req_acquire || req_get || req_prefetch) && (promoteT_normal || promoteT_L3 || promoteT_alias)

  /* ======== Task allocation ======== */
  // Theoretically, data to be released is saved in ReleaseBuffer, so Acquire can be sent as soon as req enters mshr
  io.tasks.source_a.valid := !state_dups(0).s_acquire
  io.tasks.source_b.valid := !state_dups(0).s_pprobe || !state_dups(0).s_rprobe
  val will_mp_release_valid = !state_dups(0).s_release && state_dups(0).w_rprobeacklast && !io.bMergeTask.valid &&
    state_dups(0).w_grantlast &&
    state_dups(0).w_replResp && // release after Grant to L1 sent and replRead returns
    state_dups(0).w_release
  val mp_release_valid = RegNext(will_mp_release_valid) && !io.bMergeTask.valid && will_mp_release_valid // add reg because sinkB bMergeTask out add pipe

  val mp_probeack_valid = !state_dups(0).s_probeack && state_dups(0).w_pprobeacklast && state_dups(0).w_release
  val mp_merge_probeack_valid = !state_dups(0).s_merge_probeack && state_dups(1).w_rprobeacklast  && state_dups(0).w_release
  val mp_grant_valid = !state_dups(0).s_refill && state_dups(0).w_grantlast && state_dups(1).w_rprobeacklast && state_dups(0).w_release // [Alias] grant after rprobe done
  io.tasks.mainpipe.valid := mp_release_valid || mp_probeack_valid || mp_merge_probeack_valid || mp_grant_valid
  // io.tasks.prefetchTrain.foreach(t => t.valid := !state_dups(0).s_triggerprefetch.getOrElse(true.B))


  val a_task = {
    val oa = io.tasks.source_a.bits
    oa.tag := req.tag
    oa.set := req_set_dups(0)
    oa.off := req.off
    oa.source := io.id
    oa.opcode := Mux(
      req_acquirePerm,
      req.opcode,
      // Get or AcquireBlock
      AcquireBlock
    )
    oa.param := Mux(
      req_needT,
      Mux(dirResult.hit, BtoT, NtoT),
      NtoB
    )
    oa.size := req.size
    oa.reqSource := req.reqSource
    oa
  }

  val b_task = {
    val ob = io.tasks.source_b.bits
    ob.tag := dirResult_tag_dups(0)
    ob.set := dirResult.set
    ob.off := 0.U
    ob.opcode := Probe
    ob.param := Mux(
      !state_dups(0).s_pprobe,
      req.param,
      Mux(
        req_get && dirResult.hit && meta.state === TRUNK,
        toB,
        toN
      )
    )
    ob.alias.foreach(_ := meta.alias.getOrElse(0.U))
    ob
  }
  val mp_release, mp_probeack, mp_merge_probeack, mp_grant = Wire(new TaskBundle)
  val mp_release_task = {
    mp_release.channel := req.channel
    mp_release.tag := dirResult_tag_dups(1)
    mp_release.set := req_set_dups(1)
    mp_release.off := 0.U
    mp_release.alias.foreach(_ := 0.U)
    mp_release.vaddr.foreach(_ := 0.U)
    // if dirty, we must ReleaseData
    // if accessed, we ReleaseData to keep the data in L3, for future access to be faster
    // [Access] TODO: consider use a counter
    mp_release.opcode := {
      cacheParams.releaseData match {
        case 0 => Mux(meta.dirty && meta.state =/= INVALID || probeDirty, ReleaseData, Release)
        case 1 => Mux(meta.dirty && meta.state =/= INVALID || probeDirty || meta.accessed, ReleaseData, Release)
        case 2 => Mux(meta.prefetch.getOrElse(false.B) && !meta.accessed, Release, ReleaseData) //TODO: has problem with this
        case 3 => ReleaseData // best performance with HuanCun-L3
      }
    }
    mp_release.param := Mux(isT(meta.state), TtoN, BtoN)
    mp_release.size := 0.U(msgSizeBits.W)
    mp_release.sourceId := 0.U(sourceIdBits.W)
    mp_release.bufIdx := 0.U(bufIdxBits.W)
    mp_release.needProbeAckData := false.B
    mp_release.mshrTask := true.B
    mp_release.mshrId := io.id
    mp_release.aliasTask.foreach(_ := false.B)
    // mp_release definitely read releaseBuf and refillBuf at ReqArb
    // and it needs to write refillData to DS, so useProbeData is set false according to DS.wdata logic
    mp_release.useProbeData := false.B
    mp_release.way := dirResult.way
    mp_release.pfVec.foreach(_ := PfSource.NONE)
    mp_release.needHint.foreach(_ := false.B)
    mp_release.dirty := meta.dirty && meta.state =/= INVALID || probeDirty
    mp_release.metaWen := false.B
    mp_release.meta := MetaEntry()
    mp_release.tagWen := false.B
    mp_release.dsWen := true.B
    mp_release.replTask := true.B
    mp_release.mergeTask := false.B
    mp_release.wayMask := 0.U(cacheParams.ways.W)
    mp_release.reqSource := 0.U(MemReqSource.reqSourceBits.W)
    mp_release
  }

  val mp_probeack_task = {
    mp_probeack.channel := req.channel
    mp_probeack.tag := req.tag
    mp_probeack.set := req_set_dups(2)
    mp_probeack.off := req.off
    mp_probeack.alias.foreach(_ := 0.U)
    mp_probeack.vaddr.foreach(_ := 0.U)
    mp_probeack.opcode := Mux(
      meta.dirty && isT(meta.state) || probeDirty || req.needProbeAckData,
      ProbeAckData,
      ProbeAck
    )
    mp_probeack.param := ParallelLookUp(
      Cat(isT(meta.state), req.param(bdWidth - 1, 0)),
      Seq(
        Cat(false.B, toN) -> BtoN,
        Cat(true.B, toN) -> TtoN,
        Cat(true.B, toB) -> TtoB
      )
    )
    mp_probeack.size := 0.U(msgSizeBits.W)
    mp_probeack.sourceId := 0.U(sourceIdBits.W)
    mp_probeack.bufIdx := 0.U(bufIdxBits.W)
    mp_probeack.needProbeAckData := false.B
    mp_probeack.mshrTask := true.B
    mp_probeack.mshrId := io.id
    mp_probeack.aliasTask.foreach(_ := false.B)
    mp_probeack.useProbeData := true.B // write [probeAckData] to DS, if not probed toN
    mp_probeack.way := dirResult.way
    mp_probeack.pfVec.foreach(_ := PfSource.NONE)
    mp_probeack.needHint.foreach(_ := false.B)
    mp_probeack.dirty := meta.dirty && meta.state =/= INVALID || probeDirty
    mp_probeack.meta := MetaEntry(
      dirty = false.B,
      state = Mux(
        req.param === toN,
        INVALID,
        Mux(
          req.param === toB,
          BRANCH,
          meta.state
        )
      ),
      clients = Fill(clientBits, !(probeGotN || (nestCIsXtoN && alreadyNestC))),
      alias = meta.alias, //[Alias] Keep alias bits unchanged
      prefetch = req.param =/= toN && meta_pft,
      accessed = req.param =/= toN && meta.accessed
    )
    mp_probeack.metaWen := true.B
    mp_probeack.tagWen := false.B
    mp_probeack.dsWen := req.param =/= toN && probeDirty
    mp_probeack.wayMask := 0.U(cacheParams.ways.W)
    mp_probeack.reqSource := 0.U(MemReqSource.reqSourceBits.W)
    mp_probeack.replTask := false.B
    mp_probeack.mergeTask := false.B
    mp_probeack
  }

  val mp_merge_probeack_task = {
    val task = RegEnable(io.bMergeTask.bits.task, 0.U.asTypeOf(new TaskBundle), io.bMergeTask.valid)
    mp_merge_probeack.channel := task.channel
    mp_merge_probeack.tag := task.tag
    mp_merge_probeack.set := task.set
    mp_merge_probeack.off := task.off
    mp_merge_probeack.opcode := Mux(
      meta.dirty && isT(meta.state) || probeDirty || task.needProbeAckData,
      ProbeAckData,
      ProbeAck
    )
    // TODO: has problem here
//    mp_merge_probeack.param := ParallelLookUp(
//      Cat(isT(meta.state), task.param(bdWidth - 1, 0)),
//      Seq(
//        Cat(false.B, toN) -> BtoN,
//        Cat(true.B, toN) -> TtoN,
//        Cat(true.B, toB) -> TtoB
//      )
//    )
    mp_merge_probeack.param := Mux(isT(meta.state), TtoN, BtoN)
    mp_merge_probeack.mshrTask := true.B
    mp_merge_probeack.mshrId := io.id
    // mp_merge_probeack definitely read releaseBuf and refillBuf at ReqArb
    // and it needs to write refillData to DS, so useProbeData is set false according to DS.wdata logic
    // TODO: has problem here
    mp_merge_probeack.useProbeData := false.B
    mp_merge_probeack.way := dirResult.way
    mp_merge_probeack.dirty := meta.dirty && meta.state =/= INVALID || probeDirty
    mp_merge_probeack.meta := MetaEntry(
      dirty = false.B,
      state = Mux(task.param === toN, INVALID, Mux(task.param === toB, BRANCH, meta.state)),
      clients = Fill(clientBits, !(probeGotN || (nestCIsXtoN && alreadyNestC))),
      alias = meta.alias,
      prefetch = task.param =/= toN && meta_pft,
      accessed = task.param =/= toN && meta.accessed
    )
    // TODO: has problem here
//    mp_merge_probeack.metaWen := true.B && !AneedReplMergeB
    mp_merge_probeack.metaWen := false.B
    mp_merge_probeack.tagWen := false.B
//    mp_merge_probeack.dsWen := task.param =/= toN && probeDirty
//    mp_merge_probeack.dsWen := AneedReplMergeB && alreadySRefill // === true.B
    mp_merge_probeack.dsWen := true.B
    mp_merge_probeack.mergeTask := true.B

    // unused, set to default
    mp_merge_probeack.alias.foreach(_ := 0.U)
    mp_merge_probeack.vaddr.foreach(_ := 0.U)
    mp_merge_probeack.aliasTask.foreach(_ := false.B)
    mp_merge_probeack.size := offsetBits.U
    mp_merge_probeack.sourceId := 0.U
    mp_merge_probeack.bufIdx := 0.U
    mp_merge_probeack.needProbeAckData := false.B
    mp_merge_probeack.pfVec.foreach(_ := PfSource.NONE)
    mp_merge_probeack.needHint.foreach(_ := false.B)
    mp_merge_probeack.wayMask := Fill(cacheParams.ways, "b1".U)
    mp_merge_probeack.replTask := true.B
    mp_merge_probeack.reqSource := MemReqSource.NoWhere.id.U
  }

  val mp_grant_task    = {
    mp_grant.channel := req.channel
    mp_grant.tag := req.tag
    mp_grant.set := req_set_dups(3)
    mp_grant.off := req.off
    mp_grant.sourceId := req.sourceId
    mp_grant.alias.foreach(_ := 0.U)
    mp_grant.vaddr.foreach(_ := 0.U)
    mp_grant.opcode := odOpGen(req.opcode)
    mp_grant.param := Mux(
      req_get || req_prefetch,
      0.U, // Get -> AccessAckData
      MuxLookup( // Acquire -> Grant
        req.param,
        req.param,
        Seq(
          NtoB -> Mux(req_promoteT, toT, toB),
          BtoT -> toT,
          NtoT -> toT
        )
      )
    )
    mp_grant.size := 0.U(msgSizeBits.W)
    mp_grant.bufIdx := 0.U(bufIdxBits.W)
    mp_grant.needProbeAckData := false.B
    mp_grant.mshrTask := true.B
    mp_grant.mshrId := io.id
    mp_grant.way := dirResult.way
    mp_grant.aliasTask.foreach(_ := false.B)
    // if it is a Get or Prefetch, then we must keep alias bits unchanged
    // in case future probes gets the wrong alias bits
    val aliasFinal = Mux(req_get || req_prefetch, meta.alias.getOrElse(0.U), req.alias.getOrElse(0.U))
    mp_grant.alias.foreach(_ := aliasFinal)
    mp_grant.aliasTask.foreach(_ := req.aliasTask.getOrElse(false.B))
    // [Alias] write probeData into DS for alias-caused Probe,
    // but not replacement-cased Probe
    mp_grant.useProbeData := dirResult.hit && req_get || req.aliasTask.getOrElse(false.B)
    mp_grant.dirty := false.B

    mp_grant.meta := MetaEntry(
      dirty = gotDirty || dirResult.hit && (meta.dirty || probeDirty),
      state = Mux(
        req_get,
        Mux( // Get
          dirResult.hit,
          Mux(isT(meta.state), TIP, BRANCH),
          Mux(req_promoteT, TIP, BRANCH)
        ),
        Mux( // Acquire
          req_promoteT || req_needT,
          Mux(req_prefetch, TIP, TRUNK),
          BRANCH
        )
      ),
      clients = Mux(
        req_prefetch,
        Mux(dirResult.hit, meta.clients, Fill(clientBits, false.B)),
        Fill(clientBits, !(req_get && (!dirResult.hit || meta_no_client || probeGotN || probeGotN || (nestCIsXtoN && alreadyNestC))))
      ),
      alias = Some(aliasFinal),
      prefetch = req_prefetch || dirResult.hit && meta_pft,
      accessed = req_acquire || req_get
    )
    mp_grant.metaWen := true.B
    mp_grant.tagWen := !dirResult.hit
    mp_grant.dsWen := (!dirResult.hit || gotDirty) && (gotGrantData || gotGrant) || probeDirty && (req_get || req.aliasTask.getOrElse(false.B))
    mp_grant.pfVec.foreach(_ := req.pfVec.get)
    mp_grant.needHint.foreach(_ := false.B)
    mp_grant.replTask := !dirResult.hit // Get and Alias are hit that does not need replacement
    mp_grant.mergeTask := false.B
    mp_grant.wayMask := 0.U(cacheParams.ways.W)
    mp_grant.reqSource := 0.U(MemReqSource.reqSourceBits.W)
    mp_grant
  }
  io.tasks.mainpipe.bits := ParallelPriorityMux(
    Seq(
      mp_grant_valid    -> mp_grant,
      mp_release_valid  -> mp_release,
      mp_probeack_valid -> mp_probeack,
      mp_merge_probeack_valid -> mp_merge_probeack
    )
  )
  io.tasks.mainpipe.bits.reqSource := req.reqSource

  // io.tasks.prefetchTrain.foreach {
  //   train =>
  //     train.bits.tag := req.tag
  //     train.bits.set := req_set_dups(0)
  //     train.bits.needT := req_needT
  //     train.bits.source := req.source
  // }

  /* ======== Task update ======== */
  when (io.tasks.source_a.fire) {
    state_dups.foreach(_.s_acquire := true.B)
  }
  when (io.tasks.source_b.fire) {
    state_dups.foreach(_.s_pprobe := true.B)
    state_dups.foreach(_.s_rprobe := true.B)
    alreadySendProbe := true.B
  }
  when (io.tasks.mainpipe.ready) {
    when (mp_merge_probeack_valid) {
      state_dups.foreach(_.s_merge_probeack := true.B)
    }.elsewhen (mp_grant_valid) {
      state_dups.foreach(_.s_refill := true.B)
      alreadySRefill := true.B
    }.elsewhen (mp_release_valid) {
      state_dups.foreach(_.s_release := true.B)
      meta.state := INVALID
    }.elsewhen (mp_probeack_valid) {
      state_dups.foreach(_.s_probeack := true.B)
    }
  }
  // prefetchOpt.foreach {
  //   _ =>
  //     when (io.tasks.prefetchTrain.get.fire) {
  //       state.s_triggerprefetch.get := true.B
  //     }
  // }

  /* ======== Handling response ======== */
  val c_resp = io.resps.sink_c
  val d_resp = io.resps.sink_d
  val e_resp = io.resps.sink_e
  when (c_resp.valid) {
    when (c_resp.bits.opcode === ProbeAck || c_resp.bits.opcode === ProbeAckData) {
      state_dups.foreach(_.w_rprobeackfirst := true.B)
      state_dups.foreach(_.w_rprobeacklast := state_dups(1).w_rprobeacklast || c_resp.bits.last)
      state_dups.foreach(_.w_pprobeackfirst := true.B)
      state_dups.foreach(_.w_pprobeacklast := state_dups(1).w_pprobeacklast || c_resp.bits.last)
      state_dups.foreach(_.w_pprobeack := state_dups(0).w_pprobeack || req.off === 0.U || c_resp.bits.last)
    }
    if(cacheParams.enableAssert) { assert(!(c_resp.bits.opcode === ProbeAckData && c_resp.bits.param === NtoN)) }
    when (c_resp.bits.opcode === ProbeAckData) {
      probeDirty := true.B
    }
    when (isToN(c_resp.bits.param)) {
      probeGotN := true.B
    }
  }

  when (d_resp.valid) {
    when(d_resp.bits.opcode === Grant || d_resp.bits.opcode === GrantData || d_resp.bits.opcode === AccessAck) {
      state_dups.foreach(_.w_grantfirst := true.B)
      state_dups.foreach(_.w_grantlast := d_resp.bits.last)
      state_dups.foreach(_.w_grant := req.off === 0.U || d_resp.bits.last)  // TODO? why offset?
    }
    when(d_resp.bits.opcode === Grant || d_resp.bits.opcode === GrantData) {
      gotT := d_resp.bits.param === toT
      gotDirty := gotDirty || d_resp.bits.dirty
    }
    when(d_resp.bits.opcode === GrantData) {
      gotGrantData := true.B
    }
    when(d_resp.bits.opcode === Grant) {
      gotGrant := true.B
    }
    when(d_resp.bits.opcode === ReleaseAck) {
      state_dups.foreach(_.w_releaseack := true.B)
    }
  }

  when (e_resp.valid && e_resp.bits.opcode === GrantAck) {
    state_dups.foreach(_.w_grantack := true.B)
  }.elsewhen(e_resp.valid && e_resp.bits.opcode === AccessAckData && e_resp.bits.last) {
    state_dups.foreach(_.s_accessackdata := true.B)
  }

  val replResp = io.replResp.bits
  when (io.replResp.valid && replResp.retry) {
    state_dups.foreach(_.s_refill := false.B)
  }
  when (io.replResp.valid && !replResp.retry) {
    state_dups.foreach(_.w_replResp := true.B)

    // update meta (no need to update hit/set/error/replacerInfo of dirResult)
    dirResult_tag_dups.foreach(_ := replResp.tag)
    dirResult.way := replResp.way
    dirResult.meta := replResp.meta

    // replacer choosing:
    // 1. an invalid way, release no longer needed
    // 2. the same way, just release as normal (only now we set s_release)
    // 3. differet way, we need to update meta and release that way
    // if meta has client, rprobe client
    when (replResp.meta.state =/= INVALID) {
      // set release flags
      state_dups.foreach(_.s_release := false.B)
      state_dups.foreach(_.w_releaseack := false.B)
      // rprobe clients if any
      when(replResp.meta.clients.orR) {
        state_dups.foreach(_.s_rprobe := false.B)
        state_dups.foreach(_.w_rprobeackfirst := false.B)
        state_dups.foreach(_.w_rprobeacklast := false.B)
      }
    }
  }

  when (req_valid_dups(0)) {
    timer := timer + 1.U
  }
  
  val no_schedule = state_dups(0).s_refill && state_dups(0).s_probeack && state_dups(1).s_merge_probeack && state_dups(1).s_release && state_dups(1).s_accessackdata // && state.s_triggerprefetch.getOrElse(true.B)
  val no_wait = state_dups(2).w_rprobeacklast && state_dups(2).w_pprobeacklast && state_dups(0).w_grantlast && state_dups(0).w_releaseack && state_dups(0).w_grantack && state_dups(1).w_replResp
  val will_free = no_schedule && no_wait
  when (will_free && req_valid_dups(1)) {
    req_valid_dups.foreach(_ := false.B)
    timer := 0.U
  }

  // when grant not received, B can nest A
  val nestB = !state_dups(0).w_grantfirst

  // mergeB is only allowed when release not sent
  //(TODO: or we could just blockB, since Release will be sent to MP very shortly and have no deadlock problem)
  val mergeB = !state_dups(2).s_release && !mp_release_valid
  // alias: should protect meta from being accessed or occupied
  val releaseNotSent = !state_dups(3).s_release || !state_dups(2).s_merge_probeack || io.bMergeTask.valid
  // if releaseTask is already in mainpipe_s1/s2, while a refillTask in mainpipe_s3, the refill should also be blocked and retry
  val blockRefill = releaseNotSent || RegNext(releaseNotSent, false.B) || RegNext(RegNext(releaseNotSent, false.B), false.B)
  io.status.valid := req_valid_dups(2)
  io.status.bits.channel := req.channel
  io.status.bits.set := req_set_dups(4)
  io.status.bits.reqTag := req.tag
  io.status.bits.metaTag := dirResult_tag_dups(2)
  io.status.bits.needsRepl := blockRefill
  // wait for resps, high as valid
  io.status.bits.w_c_resp := !state_dups(3).w_rprobeacklast || !state_dups(3).w_pprobeacklast || !state_dups(0).w_pprobeack
  io.status.bits.w_d_resp := !state_dups(0).w_grantlast || !state_dups(0).w_grant || !state_dups(0).w_releaseack
  io.status.bits.w_e_resp := !state_dups(0).w_grantack  || !state_dups(0).s_accessackdata
  io.status.bits.will_free := will_free
  io.status.bits.is_miss := !dirResult.hit
  io.status.bits.is_prefetch := req_prefetch
  io.status.bits.reqSource := req.reqSource

  io.msInfo.valid := req_valid_dups(3)
  io.msInfo.bits.set := req_set_dups(5)
  io.msInfo.bits.way := dirResult.way
  io.msInfo.bits.reqTag := req.tag
  io.msInfo.bits.needRelease := !state_dups(0).w_releaseack
  io.msInfo.bits.blockRefill := blockRefill
  io.msInfo.bits.dirHit := dirResult.hit
  io.msInfo.bits.metaTag := dirResult_tag_dups(3)
  io.msInfo.bits.willFree := will_free
  io.msInfo.bits.nestB := nestB
  io.msInfo.bits.mergeB := mergeB
  io.msInfo.bits.isAcqOrPrefetch := req_acquire || req_prefetch
  io.msInfo.bits.isPrefetch := req_prefetch

  if(cacheParams.enableAssert) {
    assert(!(c_resp.valid && !io.status.bits.w_c_resp))
    assert(!(d_resp.valid && !io.status.bits.w_d_resp))
    assert(!(e_resp.valid && !io.status.bits.w_e_resp))
  }

  /* ======== Handling Nested B ======== */
  when (io.bMergeTask.valid) {
    AneedReplMergeB := true.B
    state_dups.foreach(_.s_merge_probeack := false.B)
    state_dups.foreach(_.s_release := true.B)
    state_dups.foreach(_.w_releaseack := true.B)
    when (meta.clients.orR && !alreadySendProbe && state_dups(0).s_rprobe =/= false.B) {
      state_dups.foreach(_.s_rprobe := false.B)
      state_dups.foreach(_.w_rprobeackfirst := false.B)
      state_dups.foreach(_.w_rprobeacklast := false.B)
    }
  }

  // for A miss, meta == BRANCH, B nest A
  val nestedwb_match_b = req_valid_dups(4) && req_set_dups(6) === io.nestedwb.set && req.tag === io.nestedwb.tag
  when(nestedwb_match_b){
    when(io.nestedwb.b_set_meta_N && dirResult.hit && meta.state =/= INVALID){
      dirResult.hit := false.B
      state_dups.foreach(_.w_replResp := false.B)
      if(cacheParams.enableAssert) assert(meta.state =/= TIP)
    }
  }

  /* ======== Handling Nested C ======== */
  // for A miss, only when replResp do we finally choose a way, allowing nested C
  // for A-alias, always allowing nested C (state.w_replResp === true.B)
  val nestedwb_match = req_valid_dups(5) && meta.state =/= INVALID &&
    dirResult.set === io.nestedwb.set &&
    dirResult_tag_dups(4) === io.nestedwb.tag &&
    state_dups(2).w_replResp

  // when probeAck NtoN, mshr need to wait RelaseData nest it
  // Warnning: under logic only consider nest one time
  val wait_release_counter = RegInit(0.U(16.W)) // MAX = 65535
  wait_release_counter := Mux(!state_dups(0).w_release, wait_release_counter + 1.U, 0.U)

  when (io.nestedwb.is_c && nestedwb_match){
    alreadyNestC := true.B
    nestCIsXtoN := io.nestedwb.c_param === TtoN || io.nestedwb.c_param === BtoN
    state_dups.foreach(_.w_release := true.B)
  }.elsewhen(c_resp.valid){
    when(c_resp.bits.opcode === ProbeAck && c_resp.bits.param === NtoN) {
      state_dups.foreach(_.w_release := alreadyNestC) // when alreadyNestC not need to wait release
    }
  }.elsewhen(wait_release_counter === 5000.U){ // automatic unlocking because some time it will never be nest by C (For more information, see Bug #157)
    state_dups.foreach(_.w_release := true.B)
  }

  when (nestedwb_match) {
    when (io.nestedwb.c_set_dirty) {
      meta.dirty := true.B
    }
//    TODO: wait to test
//    when(req_valid_dups(5) && req.fromB){
//      when(state_dups(0).s_pprobe === false.B && !alreadySendProbe){
//        state_dups(0).s_pprobe := true.B
//        state_dups(0).w_pprobeack := true.B
//        state_dups(0).w_pprobeackfirst := true.B
//        state_dups(0).w_pprobeacklast := true.B
//      }
//    }
  }
  // let nested C write ReleaseData to the MSHRBuffer entry of this MSHR id
  // This is the VALID signal for releaseBuf.io.w(2)
  io.nestedwbData := nestedwb_match && io.nestedwb.c_set_dirty

  dontTouch(state_dups)

  /* ======== Performance counters ======== */
  // time stamp
  val acquire_ts = RegEnable(timer, false.B, io.tasks.source_a.fire)
  val probe_ts = RegEnable(timer, false.B, io.tasks.source_b.fire)
  val release_ts = RegEnable(timer, false.B, !mp_grant_valid && mp_release_valid && io.tasks.mainpipe.ready)

  val acquire_period = IO(Output(UInt(64.W)))
  val probe_period = IO(Output(UInt(64.W)))
  val release_period = IO(Output(UInt(64.W)))

  if (cacheParams.enablePerf) {
    acquire_period := timer - acquire_ts
    probe_period := timer - probe_ts
    release_period := timer - release_ts
  } else {
    acquire_period := 0.U
    probe_period := 0.U
    release_period := 0.U
  }
}
