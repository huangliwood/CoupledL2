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
import xs.utils._
import coupledL2.MetaData._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.tilelink._
import freechips.rocketchip.tilelink.TLMessages._
import freechips.rocketchip.tilelink.TLPermissions._
import coupledL2.utils._
import coupledL2.debug._
import coupledL2.prefetch.{AccessState, HyperPrefetchParams, PrefetchEvict, PrefetchTrain}
import xs.utils.perf.HasPerfLogging

class MainPipe(implicit p: Parameters) extends L2Module with HasPerfLogging with HasPerfEvents{
  val io = IO(new Bundle() {
    /* receive task from arbiter at stage 2 */
    val taskFromArb_s2 = Flipped(ValidIO(new TaskBundle()))
    /* status from arbiter at stage1  */
    val taskInfo_s1 = Flipped(ValidIO(new TaskBundle()))

    /* handle set conflict in req arb */
    val fromReqArb = Input(new Bundle() {
      val status_s1 = new PipeEntranceStatus
    })
    /* block B and C at Entrance */
    val toReqArb = Output(new BlockInfo())

    /* block A at ReqArb and ReqBuf */
    val mpInfo = Vec(2,  ValidIO(new Bundle() {
      val tag = UInt(tagBits.W)
      val set = UInt(setBits.W)
      val mshrTask = Bool()
      val metaWen = Bool()
    }))

    /* handle capacity conflict of GrantBuffer */
    val status_vec = Vec(3, ValidIO(new PipeStatus))

    /* block sinkB */
    val toSinkB = ValidIO(new Bundle() {
      val tag = UInt(tagBits.W)
      val set = UInt(setBits.W)
      val willAllocMshr = Bool()
    })

    /* get dir result at stage 3 */
    val dirResp_s3 = Input(new DirResult)
    val replResp = Flipped(ValidIO(new ReplacerResult))

    /* send task to MSHRCtl at stage 3 */
    val toMSHRCtl = new Bundle() {
      val mshr_alloc_s3 = ValidIO(new MSHRRequest())
    }

    val fromMSHRCtl = new Bundle() {
      val mshr_alloc_ptr = Input(UInt(mshrBits.W))
    }

    /* read C-channel Release Data and write into DS */
    val bufRead = Output(ValidIO(new PipeBufferRead))
    val bufResp = Input(new PipeBufferResp)

    /* get ReleaseBuffer and RefillBuffer read result */
    val refillBufResp_s3 = Flipped(ValidIO(new DSBlock))
    val releaseBufResp_s3 = Flipped(ValidIO(new DSBlock))

    /* read or write data storage */
    val toDS = new Bundle() {
      val req_s3 = ValidIO(new DSRequest)
      val rdata_s5 = Input(new DSBlock)
      val wdata_s3 = Output(new DSBlock)
    }

    /* send Release/Grant/ProbeAck via SourceC/D channels */
    val toSourceC, toSourceD = DecoupledIO(new Bundle() {
      val task = new TaskBundle
      val data = new DSBlock
    })

    /* write dir, including reset dir */
    val metaWReq = ValidIO(new MetaWrite)
    val tagWReq = ValidIO(new TagWrite)

    /* read DS and write data into ReleaseBuf when the task needs to replace */
    val releaseBufWrite = Flipped(new MSHRBufWrite()) // s5 & s6

    /* read DS and write data into RefillBuf when Acquire toT hits on B */
    val refillBufWrite = Flipped(new MSHRBufWrite())

    val nestedwb = Output(new NestedWriteback)
    val nestedwbData = Output(new DSBlock)

    /* send prefetchTrain to Prefetch to trigger a prefetch req */
    val prefetchTrain = prefetchOpt.map(_ => DecoupledIO(new PrefetchTrain))
    val prefetchEvict = if(prefetchOpt.isDefined){
      prefetchOpt.get match{
        case hyper: HyperPrefetchParams => Some(DecoupledIO(new PrefetchEvict))
        case _ => None
      }
    } else {
      None
    }
    val toMonitor = Output(new MainpipeMoni())
  })

  val resetFinish = RegInit(false.B)
  val resetIdx = RegInit((cacheParams.sets - 1).U)
  /* block reqs when reset */
  when(!resetFinish) {
    resetIdx := resetIdx - 1.U
  }
  when(resetIdx === 0.U) {
    resetFinish := true.B
  }

  val c_s3, c_s4, c_s5 = Wire(io.toSourceC.cloneType)
  val d_s3, d_s4, d_s5 = Wire(io.toSourceD.cloneType)

  /* ======== Stage 2 ======== */
  // send out MSHR task if data is not needed
  val task_s2 = io.taskFromArb_s2
  val hasData_s2 = task_s2.bits.opcode(0)

  io.bufRead.valid := task_s2.valid && task_s2.bits.fromC && task_s2.bits.opcode(0)
  io.bufRead.bits.bufIdx := task_s2.bits.bufIdx

  /* ======== Stage 3 ======== */
  val task_s3 = RegInit(0.U.asTypeOf(Valid(new TaskBundle())))
  task_s3.valid := task_s2.valid
  when(task_s2.valid) {
    task_s3.bits := task_s2.bits
  }

  /* ======== Enchantment ======== */
  val dirResult_s3    = io.dirResp_s3
  val meta_s3         = dirResult_s3.meta
  val req_s3          = task_s3.bits

  val mshr_req_s3     = req_s3.mshrTask
  val sink_req_s3     = !mshr_req_s3
  val sinkA_req_s3    = !mshr_req_s3 && req_s3.fromA
  val sinkB_req_s3    = !mshr_req_s3 && req_s3.fromB
  val sinkC_req_s3    = !mshr_req_s3 && req_s3.fromC

  val req_acquire_s3        = sinkA_req_s3 && (req_s3.opcode === AcquireBlock || req_s3.opcode === AcquirePerm)
  val req_acquireBlock_s3   = sinkA_req_s3 && req_s3.opcode === AcquireBlock
  val req_prefetch_s3       = sinkA_req_s3 && req_s3.opcode === Hint
  val req_get_s3            = sinkA_req_s3 && req_s3.opcode === Get

  val mshr_grant_s3         = mshr_req_s3 && req_s3.fromA && req_s3.opcode(2, 1) === Grant(2, 1) // Grant or GrantData from mshr
  val mshr_grantdata_s3     = mshr_req_s3 && req_s3.fromA && req_s3.opcode === GrantData
  val mshr_accessackdata_s3 = mshr_req_s3 && req_s3.fromA && req_s3.opcode === AccessAckData
  val mshr_hintack_s3       = mshr_req_s3 && req_s3.fromA && req_s3.opcode === HintAck
  val mshr_probeack_s3      = mshr_req_s3 && req_s3.fromB && req_s3.opcode(2, 1) === ProbeAck(2, 1) // ProbeAck or ProbeAckData from mshr
  val mshr_probeackdata_s3  = mshr_req_s3 && req_s3.fromB && req_s3.opcode === ProbeAckData
  val mshr_release_s3       = mshr_req_s3 && req_s3.opcode(2, 1) === Release(2, 1) // voluntary Release or ReleaseData from mshr

  val meta_has_clients_s3   = meta_s3.clients.orR
  val req_needT_s3          = needT(req_s3.opcode, req_s3.param) // require T status to handle req

  //[Alias] TODO: consider 1 client for now
  val cache_alias           = req_acquire_s3 && dirResult_s3.hit && meta_s3.clients(0) &&
                              meta_s3.alias.getOrElse(0.U) =/= req_s3.alias.getOrElse(0.U)

  val mshr_refill_s3 = (mshr_accessackdata_s3 || mshr_hintack_s3 || mshr_grant_s3) // needs refill to L2 DS
  val retry = io.replResp.valid && io.replResp.bits.retry
  val need_repl = io.replResp.valid && io.replResp.bits.meta.state =/= INVALID && req_s3.replTask // Grant needs replacement

  val a_need_replacement = mshr_refill_s3 && need_repl && !retry // b and c do not need replacement

  /* ======== Interact with MSHR ======== */
  val acquire_on_miss_s3  = req_acquire_s3 || req_prefetch_s3 || req_get_s3 // TODO: remove this cause always acquire on miss?
  val acquire_on_hit_s3   = meta_s3.state === BRANCH && req_needT_s3
  // For channel A reqs, alloc mshr when: acquire downwards is needed || alias
  val need_acquire_s3_a   = req_s3.fromA && Mux(
    dirResult_s3.hit,
    acquire_on_hit_s3,
    acquire_on_miss_s3
  )
  val need_probe_s3_a = req_get_s3 && dirResult_s3.hit && meta_s3.state === TRUNK

  val need_mshr_s3_a = need_acquire_s3_a || need_probe_s3_a || cache_alias
  // For channel B reqs, alloc mshr when Probe hits in both self and client dir
  val need_mshr_s3_b = dirResult_s3.hit && req_s3.fromB &&
    !(meta_s3.state === BRANCH && req_s3.param === toB) &&
    meta_has_clients_s3

  // For channel C reqs, Release will always hit on MainPipe, no need for MSHR
  val need_mshr_s3 = need_mshr_s3_a || need_mshr_s3_b

  // // For evict address
  // val need_evict = acquire_on_miss_s3 && (!dirResult_s3.hit ) && meta_s3.state =/= INVALID
  /* Signals to MSHR Ctl */
  // Allocation of MSHR: new request only
  val alloc_state = WireInit(0.U.asTypeOf(new FSMState()))
  alloc_state.elements.foreach(_._2 := true.B)
  io.toMSHRCtl.mshr_alloc_s3.valid := task_s3.valid && !mshr_req_s3 && need_mshr_s3
  io.toMSHRCtl.mshr_alloc_s3.bits.dirResult := dirResult_s3
  io.toMSHRCtl.mshr_alloc_s3.bits.state := alloc_state

  val ms_task = io.toMSHRCtl.mshr_alloc_s3.bits.task
  ms_task.channel          := req_s3.channel
  ms_task.set              := req_s3.set
  ms_task.tag              := req_s3.tag
  ms_task.off              := req_s3.off
  ms_task.alias.foreach(_  := req_s3.alias.getOrElse(0.U))
  ms_task.vaddr.foreach(_  := req_s3.vaddr.getOrElse(0.U))
  ms_task.opcode           := req_s3.opcode
  ms_task.param            := req_s3.param
  ms_task.size             := req_s3.size
  ms_task.sourceId         := req_s3.sourceId
  ms_task.bufIdx           := 0.U(bufIdxBits.W)
  ms_task.needProbeAckData := req_s3.needProbeAckData
  ms_task.mshrTask         := false.B
  ms_task.mshrId           := 0.U(mshrBits.W)
  ms_task.aliasTask.foreach(_ := cache_alias)
  ms_task.useProbeData     := false.B
  ms_task.pfVec.foreach    (_ := req_s3.pfVec.get)
  ms_task.needHint.foreach(_  := req_s3.needHint.get)
  ms_task.dirty            := false.B
  ms_task.way              := req_s3.way
  ms_task.meta             := 0.U.asTypeOf(new MetaEntry)
  ms_task.metaWen          := false.B
  ms_task.tagWen           := false.B
  ms_task.dsWen            := false.B
  ms_task.wayMask          := 0.U(cacheParams.ways.W)
  ms_task.replTask         := false.B
  ms_task.mergeTask        := false.B
  ms_task.reqSource        := req_s3.reqSource

  /* ======== Resps to SinkA/B/C Reqs ======== */
  val sink_resp_s3 = WireInit(0.U.asTypeOf(Valid(new TaskBundle))) // resp for sinkA/B/C request that does not need to alloc mshr
  val sink_resp_s3_a_promoteT = dirResult_s3.hit && isT(meta_s3.state)

  sink_resp_s3.valid := task_s3.valid && !mshr_req_s3 && !need_mshr_s3
  sink_resp_s3.bits := task_s3.bits
  sink_resp_s3.bits.mshrId := (1 << (mshrBits-1)).U + sink_resp_s3.bits.sourceId // extra id for reqs that do not enter mshr

  when(req_s3.fromA) {
    sink_resp_s3.bits.opcode := odOpGen(req_s3.opcode)
    sink_resp_s3.bits.param  := Mux(
      req_acquire_s3,
      Mux(req_s3.param === NtoB && !sink_resp_s3_a_promoteT, toB, toT),
      0.U // reserved
    )
  }.elsewhen(req_s3.fromB) {
    sink_resp_s3.bits.opcode := Mux(
      dirResult_s3.hit && (meta_s3.state === TIP && meta_s3.dirty || req_s3.needProbeAckData),
      ProbeAckData,
      ProbeAck
    )
    sink_resp_s3.bits.param  := Mux(!dirResult_s3.hit, NtoN,
      MuxLookup(Cat(req_s3.param, meta_s3.state), BtoB, Seq(
        Cat(toN, BRANCH) -> BtoN,
        Cat(toN, TIP)    -> TtoN,
        Cat(toB, TIP)    -> TtoB,
        Cat(toT, TIP)    -> TtoT
      )) // other combinations should miss or have mshr allocated
    )
  }.otherwise { // req_s3.fromC
    sink_resp_s3.bits.opcode := ReleaseAck
    sink_resp_s3.bits.param  := 0.U // param of ReleaseAck must be 0
  }

  val source_req_s3 = Wire(new TaskBundle)
  source_req_s3 := Mux(!mshr_req_s3, sink_resp_s3.bits, req_s3) // sink_req->resp, mshr_resp->resp

  //  TODO: debug address consider multi-bank
  def restoreAddr(set: UInt, tag: UInt) = {
    (set << offsetBits).asUInt + (tag << (setBits + offsetBits)).asUInt
  }
  val debug_addr_s3 = restoreAddr(task_s3.bits.set, task_s3.bits.tag) // (task_s3.bits.set << offsetBits).asUInt + (task_s3.bits.tag << (setBits + offsetBits)).asUInt
  dontTouch(debug_addr_s3)

  /* ======== Interact with DS ======== */
  val data_s3 = Mux(io.releaseBufResp_s3.valid, io.releaseBufResp_s3.bits.data, io.refillBufResp_s3.bits.data) // releaseBuf prior
  val c_releaseData_s3 = RegNext(io.bufResp.data.asUInt)
  val hasData_s3 = source_req_s3.opcode(0)

  val need_data_a  = dirResult_s3.hit && (req_get_s3 || req_acquireBlock_s3)
  val need_data_b  = sinkB_req_s3 && dirResult_s3.hit &&
                       (meta_s3.state === TRUNK || meta_s3.state === TIP && meta_s3.dirty || req_s3.needProbeAckData)
  val need_data_mshr_repl = mshr_refill_s3 && need_repl && !retry
  val ren                 = need_data_a || need_data_b || need_data_mshr_repl

  val wen_c = sinkC_req_s3 && isParamFromT(req_s3.param) && req_s3.opcode(0) && dirResult_s3.hit
  val wen_mshr = req_s3.dsWen && (
    mshr_probeack_s3 || mshr_release_s3 ||
    mshr_refill_s3 && !need_repl && !retry || req_s3.mergeTask
  )
  val wen   = wen_c || wen_mshr

  io.toDS.req_s3.valid    := task_s3.valid && (ren || wen)
  io.toDS.req_s3.bits.way := Mux(mshr_refill_s3 && req_s3.replTask, io.replResp.bits.way,
    Mux(mshr_req_s3, req_s3.way, dirResult_s3.way))
  io.toDS.req_s3.bits.set := Mux(mshr_req_s3, req_s3.set, dirResult_s3.set)
  io.toDS.req_s3.bits.wen := wen
  io.toDS.wdata_s3.data := Mux(
    !mshr_req_s3,
    c_releaseData_s3, // Among all sinkTasks, only C-Release writes DS
    Mux(
      req_s3.useProbeData,
      io.releaseBufResp_s3.bits.data,
      io.refillBufResp_s3.bits.data
    )
  )

  /* ======== Read DS and store data in Buffer ======== */
  // A: need_write_releaseBuf indicates that DS should be read and the data will be written into ReleaseBuffer
  //    need_write_releaseBuf is assigned true when:
  //    inner clients' data is needed, but whether the client will ack data is uncertain, so DS data is also needed, or
  val need_write_releaseBuf = need_probe_s3_a ||
    cache_alias ||
    need_data_b && need_mshr_s3_b ||
    need_data_mshr_repl
  // B: need_write_refillBuf when L1 AcquireBlock BtoT
  //    L2 sends AcquirePerm to L3, so GrantData to L1 needs to read DS ahead of time and store in RefillBuffer
  // TODO: how about AcquirePerm BtoT interaction with refill buffer?
  // !!TODO June 22th: this is no longer useful, cuz we only AcquirePerm when L1 AcquirePerm (see MSHR)
  val need_write_refillBuf = sinkA_req_s3 && req_needT_s3 && dirResult_s3.hit && meta_s3.state === BRANCH && !req_prefetch_s3

  /* ======== Write Directory ======== */
  val metaW_valid_s3_a    = sinkA_req_s3 && !need_mshr_s3_a && !req_get_s3 && !req_prefetch_s3 // get & prefetch that hit will not write meta
  val metaW_valid_s3_b    = sinkB_req_s3 && !need_mshr_s3_b && dirResult_s3.hit && (meta_s3.state === TIP || meta_s3.state === BRANCH && req_s3.param === toN)
  val metaW_valid_s3_c    = sinkC_req_s3 && dirResult_s3.hit
  val metaW_valid_s3_mshr = mshr_req_s3 && req_s3.metaWen && !(mshr_refill_s3 && retry)
  require(clientBits == 1)

  // Get and Prefetch should not change alias bit
  val metaW_s3_a_alias = Mux(
    req_get_s3 || req_prefetch_s3,
    meta_s3.alias.getOrElse(0.U),
    req_s3.alias.getOrElse(0.U)
  )
  val metaW_s3_a = MetaEntry(
    dirty = meta_s3.dirty,
    state = Mux(req_needT_s3 || sink_resp_s3_a_promoteT, TRUNK, meta_s3.state),
    clients = Fill(clientBits, true.B),
    alias = Some(metaW_s3_a_alias),
    accessed = true.B,
    prefetch = if(meta_s3.prefetch.isDefined){
      Mux(meta_s3.prefetch.get && dirResult_s3.hit, false.B, meta_s3.prefetch.get)
    } else {
      false.B
    }
  )
  val metaW_s3_b = Mux(req_s3.param === toN, MetaEntry(),
    MetaEntry(
      dirty = false.B,
      state = BRANCH,
      clients = meta_s3.clients,
      alias = meta_s3.alias,
      accessed = meta_s3.accessed
    )
  )

  val metaW_s3_c = MetaEntry(
    dirty = meta_s3.dirty || wen_c,
    state = Mux(isParamFromT(req_s3.param), TIP, meta_s3.state),
    clients = Fill(clientBits, !isToN(req_s3.param)),
    alias = meta_s3.alias,
    accessed = meta_s3.accessed
  )
  val metaW_s3_mshr = req_s3.meta

  val metaW_way = Mux(mshr_refill_s3 && req_s3.replTask, io.replResp.bits.way, // grant always use replResp way
    Mux(mshr_req_s3, req_s3.way, dirResult_s3.way))

  io.metaWReq.valid      := !resetFinish || task_s3.valid && (metaW_valid_s3_a || metaW_valid_s3_b || metaW_valid_s3_c || metaW_valid_s3_mshr)
  io.metaWReq.bits.set   := Mux(resetFinish, req_s3.set, resetIdx)
  io.metaWReq.bits.wayOH := Mux(resetFinish, UIntToOH(metaW_way), Fill(cacheParams.ways, true.B))
  io.metaWReq.bits.wmeta := Mux(
    resetFinish,
    ParallelPriorityMux(
      Seq(metaW_valid_s3_a, metaW_valid_s3_b, metaW_valid_s3_c, metaW_valid_s3_mshr),
      Seq(metaW_s3_a, metaW_s3_b, metaW_s3_c, metaW_s3_mshr)
    ),
    MetaEntry()
  )
  io.metaWReq.bits.channel := task_s3.bits.channel

  io.tagWReq.valid     := task_s3.valid && req_s3.tagWen && mshr_refill_s3 && !retry
  io.tagWReq.bits.set  := req_s3.set
  io.tagWReq.bits.way  := Mux(mshr_refill_s3 && req_s3.replTask, io.replResp.bits.way, req_s3.way)
  io.tagWReq.bits.wtag := req_s3.tag

  /* ======== Interact with Channels (C & D) ======== */
  // do not need s4 & s5
  val chnl_fire_s3 = c_s3.fire || d_s3.fire
  val req_drop_s3 = !need_write_releaseBuf && !need_write_refillBuf && (
    !mshr_req_s3 && need_mshr_s3 || chnl_fire_s3
  ) || (mshr_refill_s3 && retry)

  val data_unready_s3 = hasData_s3 && !mshr_req_s3
  val isC_s3 = Mux(
    mshr_req_s3,
    mshr_release_s3 || mshr_probeack_s3,
    req_s3.fromB && !need_mshr_s3 && !data_unready_s3
  )
  val isD_s3 = Mux(
    mshr_req_s3,
    mshr_refill_s3 && !retry,
    req_s3.fromC || req_s3.fromA && !need_mshr_s3 && !data_unready_s3
  )
  c_s3.valid := task_s3.valid && isC_s3
  d_s3.valid := task_s3.valid && isD_s3
  c_s3.bits.task      := source_req_s3
  c_s3.bits.data.data := data_s3
  d_s3.bits.task      := source_req_s3
  d_s3.bits.data.data := data_s3

  /* ======== nested & prefetch ======== */
  io.nestedwb.set := req_s3.set
  io.nestedwb.tag := req_s3.tag
  // b_set_meta_N is true if Probe toN
  io.nestedwb.b_set_meta_N := task_s3.valid && task_s3.bits.fromB && task_s3.bits.param === toN
  // This serves as VALID signal
  // c_set_dirty is true iff Release has Data
  io.nestedwb.c_set_dirty := task_s3.valid && task_s3.bits.fromC && task_s3.bits.opcode === ReleaseData

  io.nestedwbData := c_releaseData_s3.asTypeOf(new DSBlock)

  if(io.prefetchTrain.isDefined){
    val train = io.prefetchTrain.get
    train.valid := task_s3.valid && (req_acquire_s3 || req_get_s3) && req_s3.needHint.getOrElse(false.B)
    train.bits.tag := req_s3.tag
    train.bits.set := req_s3.set
    train.bits.needT := req_needT_s3
    train.bits.source := req_s3.sourceId
    train.bits.vaddr.foreach(_ := req_s3.vaddr.getOrElse(0.U))
    train.bits.state:= Mux(!dirResult_s3.hit, AccessState.MISS,
      Mux(!meta_s3.prefetch.get, AccessState.HIT, AccessState.PREFETCH_HIT))
    train.bits.pfVec := Mux(!dirResult_s3.hit, PfSource.BOP_SPP, req_s3.pfVec.getOrElse(PfSource.NONE))
  }
  if(io.prefetchEvict.isDefined){
    val evict = io.prefetchEvict.get
    evict.bits.tag := ms_task.tag
    evict.bits.set := ms_task.set
    evict.bits.is_prefetch := meta_s3.prefetch.get
    evict.valid := a_need_replacement
  }

  /* ======== Stage 4 ======== */
  val task_s4 = RegInit(0.U.asTypeOf(Valid(new TaskBundle())))
  val data_unready_s4 = RegInit(false.B)
  val data_s4 = Reg(UInt((blockBytes * 8).W))
  val ren_s4 = RegInit(false.B)
  val need_write_releaseBuf_s4 = RegInit(false.B)
  val need_write_refillBuf_s4 = RegInit(false.B)
  val isC_s4, isD_s4 = RegInit(false.B)
  task_s4.valid := task_s3.valid && !req_drop_s3
  when (task_s3.valid && !req_drop_s3) {
    task_s4.bits := Mux(sink_resp_s3.valid, source_req_s3, req_s3)
    task_s4.bits.mshrId := Mux(!task_s3.bits.mshrTask && need_mshr_s3, io.fromMSHRCtl.mshr_alloc_ptr, source_req_s3.mshrId)
    data_unready_s4 := data_unready_s3
    data_s4 := data_s3
    ren_s4 := ren
    need_write_releaseBuf_s4 := need_write_releaseBuf
    need_write_refillBuf_s4 := need_write_refillBuf
    isC_s4 := isC_s3
    isD_s4 := isD_s3
  }

  // A-alias-Acquire should send neither C nor D
//  val isC_s4 = task_s4.bits.opcode(2, 1) === Release(2, 1) && task_s4.bits.fromA && !RegNext(cache_alias, false.B) ||
//               task_s4.bits.opcode(2, 1) === ProbeAck(2, 1) && task_s4.bits.fromB
//  val isD_s4 = task_s4.bits.fromC || task_s4.bits.fromA && (
//                task_s4.bits.opcode(2, 1) === Grant(2, 1) ||
//                task_s4.bits.opcode(2, 1) === AccessAck(2, 1) ||
//                task_s4.bits.opcode === HintAck)

  // for reqs that CANNOT give response in MainPipe, but needs to write releaseBuf/refillBuf
  // we cannot drop them at s3, we must let them go to s4/s5
  val chnl_fire_s4 = c_s4.fire || d_s4.fire
  val req_drop_s4 = !need_write_releaseBuf_s4 && !need_write_refillBuf_s4 && chnl_fire_s4

  val c_d_valid_s4 = task_s4.valid && !RegNext(chnl_fire_s3, false.B)
  c_s4.valid := c_d_valid_s4 && isC_s4
  d_s4.valid := c_d_valid_s4 && isD_s4
  c_s4.bits.task := task_s4.bits
  c_s4.bits.data.data := data_s4
  d_s4.bits.task := task_s4.bits
  d_s4.bits.data.data := data_s4

  /* ======== Stage 5 ======== */
  val task_s5 = RegInit(0.U.asTypeOf(Valid(new TaskBundle())))
  val task_s5_dups_valid = RegInit(VecInit(Seq.fill(4)(false.B)))
  val ren_s5 = RegInit(false.B)
  val data_s5 = Reg(UInt((blockBytes * 8).W))
  val need_write_releaseBuf_s5_dups = RegInit(VecInit(Seq.fill(mshrsAll)(false.B)))
  val need_write_refillBuf_s5_dups = RegInit(VecInit(Seq.fill(mshrsAll)(false.B)))
  val isC_s5, isD_s5 = RegInit(false.B)
//  task_s5.valid := task_s4.valid && !req_drop_s4
  task_s5_dups_valid.foreach(_ := task_s4.valid && !req_drop_s4)
  when (task_s4.valid && !req_drop_s4) {
    task_s5.bits := task_s4.bits
    ren_s5 := ren_s4
    data_s5 := data_s4
    need_write_releaseBuf_s5_dups.foreach(_ := need_write_releaseBuf_s4)
    need_write_refillBuf_s5_dups.foreach(_ := need_write_refillBuf_s4)
    isC_s5 := isC_s4 || task_s4.bits.fromB && !task_s4.bits.mshrTask && task_s4.bits.opcode === ProbeAckData
    isD_s5 := isD_s4 || task_s4.bits.fromA && !task_s4.bits.mshrTask &&
      (task_s4.bits.opcode === GrantData || task_s4.bits.opcode === AccessAckData)
  }
  val rdata_s5 = io.toDS.rdata_s5.data
  val out_data_s5 = Mux(!task_s5.bits.mshrTask, rdata_s5, data_s5)
  val chnl_fire_s5 = c_s5.fire || d_s5.fire

  io.releaseBufWrite.valid_dups.zipWithIndex.foreach{
    case (valid, i) =>
      valid := task_s5_dups_valid(0) && need_write_releaseBuf_s5_dups(i)
  }
  io.releaseBufWrite.beat_sel   := Fill(beatSize, 1.U(1.W))
  io.releaseBufWrite.data.data  := rdata_s5
  io.releaseBufWrite.id         := task_s5.bits.mshrId
  if(cacheParams.enableAssert)
    assert(!(io.releaseBufWrite.valid_dups.reduce(_||_) && !io.releaseBufWrite.ready), "releaseBuf should be ready when given valid")

  io.refillBufWrite.valid_dups.zipWithIndex.foreach{
    case (valid, i) =>
      valid := task_s5_dups_valid(1) && need_write_refillBuf_s5_dups(i)
  }
  io.refillBufWrite.beat_sel  := Fill(beatSize, 1.U(1.W))
  io.refillBufWrite.data.data := rdata_s5
  io.refillBufWrite.id        := task_s5.bits.mshrId
  if(cacheParams.enableAssert)
    assert(!(io.refillBufWrite.valid_dups.reduce(_||_) && !io.refillBufWrite.ready), "refillBuf should be ready when given valid")

  val c_d_valid_s5 = task_s5_dups_valid(2) && !RegNext(chnl_fire_s4, false.B) && !RegNextN(chnl_fire_s3, 2, Some(false.B))
  c_s5.valid := c_d_valid_s5 && isC_s5
  d_s5.valid := c_d_valid_s5 && isD_s5
  c_s5.bits.task := task_s5.bits
  c_s5.bits.data.data := out_data_s5
  d_s5.bits.task := task_s5.bits
  d_s5.bits.data.data := out_data_s5

  /* ======== BlockInfo ======== */
  // if s2/s3 might write Dir, we must block s1 sink entrance
  // TODO:[Check] it seems that s3 Dir write will naturally block all s1 by dirRead.ready
  //        (an even stronger blocking than set blocking)
  //         so we might not need s3 blocking here
  def s23Block(chn: Char, s: TaskBundle): Bool = {
    val s1 = io.fromReqArb.status_s1
    val s1_set = chn match {
      case 'a' => s1.a_set
      case 'b' => s1.b_set
      case 'c' => s1.c_set
      case 'g' => s1.g_set
    }
    s.set === s1_set && !(s.mshrTask && !s.metaWen) // if guaranteed not to write meta, no blocking needed
  }
  def bBlock(s: TaskBundle, tag: Boolean = false): Bool = {
    val s1 = io.fromReqArb.status_s1
    // tag true: compare tag + set
    s.set === s1.b_set && (if(tag) s.tag === s1.b_tag else true.B)
  }

  io.toReqArb.blockC_s1 := task_s2.valid && s23Block('c', task_s2.bits)

  io.toReqArb.blockB_s1 :=
    task_s2.valid && bBlock(task_s2.bits) ||
    task_s3.valid && bBlock(task_s3.bits) ||
    task_s4.valid && bBlock(task_s4.bits, tag = true) ||
    task_s5_dups_valid(3) && bBlock(task_s5.bits, tag = true)

  io.toReqArb.blockA_s1 := false.B // mainpipe blockA logic in reqBuf

  io.mpInfo(0).valid := task_s2.valid
  io.mpInfo(0).bits.set := task_s2.bits.set
  io.mpInfo(0).bits.tag := task_s2.bits.tag
  io.mpInfo(0).bits.mshrTask := task_s2.bits.mshrTask
  io.mpInfo(0).bits.metaWen := task_s2.bits.metaWen
  io.mpInfo(1).valid := task_s3.valid
  io.mpInfo(1).bits.set := task_s3.bits.set
  io.mpInfo(1).bits.tag := task_s3.bits.tag
  io.mpInfo(1).bits.mshrTask := task_s3.bits.mshrTask
  io.mpInfo(1).bits.metaWen := task_s3.bits.metaWen

  io.toReqArb.blockG_s1 := task_s2.valid && s23Block('g', task_s2.bits)
  /* ======== Pipeline Status ======== */
  require(io.status_vec.size == 3)
  io.status_vec(0).valid := task_s3.valid && Mux(
    mshr_req_s3,
    mshr_refill_s3 && !retry,
    true.B
    // TODO: To consider grantBuffer capacity conflict,
    // only " req_s3.fromC || req_s3.fromA && !need_mshr_s3 " is needed
    // But to consider mshrFull, all channel_reqs are needed
  )
  io.status_vec(0).bits.channel := task_s3.bits.channel
  io.status_vec(1).valid        := task_s4.valid && isD_s4 && !need_write_releaseBuf_s4 && !need_write_refillBuf_s4
  io.status_vec(1).bits.channel := task_s4.bits.channel
  io.status_vec(2).valid        := d_s5.valid
  io.status_vec(2).bits.channel := task_s5.bits.channel

  io.toSinkB.valid := task_s3.valid
  io.toSinkB.bits.tag := task_s3.bits.tag
  io.toSinkB.bits.set := task_s3.bits.set
  io.toSinkB.bits.willAllocMshr := io.toMSHRCtl.mshr_alloc_s3.valid

  /* ======== Other Signals Assignment ======== */
  // Initial state assignment
  // ! Caution: s_ and w_ are false-as-valid
  when(req_s3.fromA) {
    alloc_state.s_refill := false.B
    alloc_state.w_grantack := req_prefetch_s3 || req_get_s3
    alloc_state.w_replResp := dirResult_s3.hit // need replRead when NOT dirHit
    // need Acquire downwards
    when(need_acquire_s3_a) {
      alloc_state.s_acquire := false.B
      alloc_state.w_grantfirst := false.B
      alloc_state.w_grantlast := false.B
      alloc_state.w_grant := false.B
    }
    // need Probe for alias
    // need Probe when Get hits on a TRUNK block
    when(cache_alias || need_probe_s3_a) {
      alloc_state.s_rprobe := false.B
      alloc_state.w_rprobeackfirst := false.B
      alloc_state.w_rprobeacklast := false.B
    }
    // need trigger a prefetch, send PrefetchTrain msg to Prefetcher
    // prefetchOpt.foreach {_ =>
    //   when (req_s3.fromA && req_s3.needHint.getOrElse(false.B) && (!dirResult_s3.hit || meta_s3.prefetch.get)) {
    //     alloc_state.s_triggerprefetch.foreach(_ := false.B)
    //   }
    // }
  }
  when(req_s3.fromB) {
    // Only consider the situation when mshr needs to be allocated
    alloc_state.s_pprobe := false.B
    alloc_state.w_pprobeackfirst := false.B
    alloc_state.w_pprobeacklast := false.B
    alloc_state.w_pprobeack := false.B
    alloc_state.s_probeack := false.B
  }

  val c = Seq(c_s5, c_s4, c_s3)
  val d = Seq(d_s5, d_s4, d_s3)
  // DO NOT use TLArbiter because TLArbiter will send continuous beats for the same source
  val c_arb = Module(new Arbiter(io.toSourceC.bits.cloneType, c.size))
  val d_arb = Module(new Arbiter(io.toSourceD.bits.cloneType, d.size))
  c_arb.io.in <> c
  d_arb.io.in <> d

  io.toSourceC <> c_arb.io.out
  io.toSourceD <> d_arb.io.out

  // Performance counters
  val hit_s3 = task_s3.valid && !mshr_req_s3 && dirResult_s3.hit
  val miss_s3 = task_s3.valid && !mshr_req_s3 && !dirResult_s3.hit
  
  if(cacheParams.enablePerf) {
    // num of mshr req
    XSPerfAccumulate("mshr_grant_req", task_s3.valid && mshr_grant_s3 && !retry)
    XSPerfAccumulate("mshr_grantdata_req", task_s3.valid && mshr_grantdata_s3 && !retry)
    XSPerfAccumulate("mshr_accessackdata_req", task_s3.valid && mshr_accessackdata_s3 && !retry)
    XSPerfAccumulate("mshr_hintack_req", task_s3.valid && mshr_hintack_s3 && !retry)
    XSPerfAccumulate("mshr_probeack_req", task_s3.valid && mshr_probeack_s3)
    XSPerfAccumulate("mshr_probeackdata_req", task_s3.valid && mshr_probeackdata_s3)
    XSPerfAccumulate("mshr_release_req", task_s3.valid && mshr_release_s3)

    // directory access result
    XSPerfAccumulate(cacheParams.name+"_a_req_hit", hit_s3 && req_s3.fromA)
    XSPerfAccumulate(cacheParams.name+"_acquire_hit", hit_s3 && req_s3.fromA &&
      (req_s3.opcode === AcquireBlock || req_s3.opcode === AcquirePerm))
    XSPerfAccumulate(cacheParams.name+"_get_hit", hit_s3 && req_s3.fromA && req_s3.opcode === Get)
    XSPerfAccumulate(cacheParams.name+"_retry", mshr_refill_s3 && retry)

    XSPerfAccumulate(cacheParams.name+"_a_req_miss", miss_s3 && req_s3.fromA)
    XSPerfAccumulate(cacheParams.name+"_acquire_miss", miss_s3 && req_s3.fromA &&
      (req_s3.opcode === AcquireBlock || req_s3.opcode === AcquirePerm))
    XSPerfAccumulate(cacheParams.name+"_get_miss", miss_s3 && req_s3.fromA && req_s3.opcode === Get)

    XSPerfAccumulate(cacheParams.name + "_c_req_miss", miss_s3 && req_s3.fromC)
    XSPerfAccumulate(cacheParams.name + "_c_req_hit", hit_s3 && req_s3.fromC)

    XSPerfAccumulate(cacheParams.name + "_a_req_need_replacement",
      task_s3.valid && req_s3.mshrTask && a_need_replacement)
    XSPerfAccumulate(cacheParams.name + "_c_req_need_replacement",
      false.B)

    XSPerfAccumulate(cacheParams.name+"_b_req_hit", hit_s3 && req_s3.fromB)
    XSPerfAccumulate(cacheParams.name+"_b_req_miss", miss_s3 && req_s3.fromB)

    XSPerfHistogram(cacheParams.name+"_a_req_access_way", perfCnt = dirResult_s3.way,
      enable = task_s3.valid && !mshr_req_s3 && req_s3.fromA, start = 0, stop = cacheParams.ways, step = 1)
    XSPerfHistogram(cacheParams.name+"_a_req_hit_way", perfCnt = dirResult_s3.way,
      enable = hit_s3 && req_s3.fromA, start = 0, stop = cacheParams.ways, step = 1)
    XSPerfHistogram(cacheParams.name+"_a_req_miss_way_choice", perfCnt = dirResult_s3.way,
      enable = miss_s3 && req_s3.fromA, start = 0, stop = cacheParams.ways, step = 1)

    // pipeline stages for sourceC and sourceD reqs
    val sourceC_pipe_len = ParallelMux(Seq(
      c_s5.fire -> 5.U,
      c_s4.fire -> 4.U,
      c_s3.fire -> 3.U
    ))
    val sourceD_pipe_len = ParallelMux(Seq(
      d_s5.fire -> 5.U,
      d_s4.fire -> 4.U,
      d_s3.fire -> 3.U
    ))
    XSPerfHistogram("sourceC_pipeline_stages", sourceC_pipe_len,
      enable = io.toSourceC.fire, start = 3, stop = 5+1, step = 1)
    XSPerfHistogram("sourceD_pipeline_stages", sourceD_pipe_len,
      enable = io.toSourceD.fire, start = 3, stop = 5+1, step = 1)

    // XSPerfAccumulate("a_req_tigger_prefetch", io.prefetchTrain.)
    prefetchOpt.foreach {
      _ =>
        XSPerfAccumulate("a_req_trigger_prefetch", io.prefetchTrain.get.fire)
        XSPerfAccumulate("a_req_trigger_prefetch_not_ready", io.prefetchTrain.get.valid && !io.prefetchTrain.get.ready)
        XSPerfAccumulate("acquire_trigger_prefetch_on_miss", io.prefetchTrain.get.fire && req_acquire_s3 && !dirResult_s3.hit)
        XSPerfAccumulate("acquire_trigger_prefetch_on_hit_pft", io.prefetchTrain.get.fire && req_acquire_s3 && dirResult_s3.hit && meta_s3.prefetch.get)
        XSPerfAccumulate("release_all", mshr_release_s3)
        XSPerfAccumulate("release_prefetch_accessed", mshr_release_s3 && meta_s3.prefetch.get && meta_s3.accessed)
        XSPerfAccumulate("release_prefetch_not_accessed", mshr_release_s3 && meta_s3.prefetch.get && !meta_s3.accessed)
        XSPerfAccumulate("get_trigger_prefetch_on_miss", io.prefetchTrain.get.fire && req_get_s3 && !dirResult_s3.hit)
        XSPerfAccumulate("get_trigger_prefetch_on_hit_pft", io.prefetchTrain.get.fire && req_get_s3 && dirResult_s3.hit && meta_s3.prefetch.get)
    }

    XSPerfAccumulate("early_prefetch", meta_s3.prefetch.getOrElse(false.B) && !meta_s3.accessed && !dirResult_s3.hit && task_s3.valid)
  }
  
  // Monitor
  io.toMonitor.task_s2 := task_s2
  io.toMonitor.task_s3 := task_s3
  io.toMonitor.task_s4 := task_s4
  io.toMonitor.task_s5 := task_s5
  io.toMonitor.dirResult_s3 := dirResult_s3
  io.toMonitor.allocMSHR_s3.valid := io.toMSHRCtl.mshr_alloc_s3.valid
  io.toMonitor.allocMSHR_s3.bits  := io.fromMSHRCtl.mshr_alloc_ptr
  io.toMonitor.metaW_s3 := io.metaWReq

  // TODO: perfEvents
  val perfEvents = Seq(
    (cacheParams.name+"_a_req_hit",  hit_s3 && req_s3.fromA), 
    (cacheParams.name+"_a_req_miss", miss_s3 && req_s3.fromA),
    (cacheParams.name+"_b_req_hit", hit_s3 && req_s3.fromB),
    (cacheParams.name+"_b_req_miss", miss_s3 && req_s3.fromB),
    (cacheParams.name + "_c_req_miss", miss_s3 && req_s3.fromC),
    (cacheParams.name + "_c_req_hit", hit_s3 && req_s3.fromC),
    (cacheParams.name+"_acquire_hit", hit_s3 && req_s3.fromA && (req_s3.opcode === AcquireBlock || req_s3.opcode === AcquirePerm)),
    (cacheParams.name+"_acquire_miss", miss_s3 && req_s3.fromA && (req_s3.opcode === AcquireBlock || req_s3.opcode === AcquirePerm)),
    (cacheParams.name+"_get_hit", hit_s3 && req_s3.fromA && req_s3.opcode === Get),
    (cacheParams.name+"_get_miss", miss_s3 && req_s3.fromA && req_s3.opcode === Get),
    (cacheParams.name+"_retry", mshr_refill_s3 && retry)
  )
  generatePerfEvent()
}
