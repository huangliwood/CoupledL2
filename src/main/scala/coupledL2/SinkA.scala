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
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.tilelink._
import freechips.rocketchip.tilelink.TLMessages._
import freechips.rocketchip.tilelink.TLHints._
import coupledL2.prefetch.PrefetchReq
import xs.utils.Pipeline
import xs.utils.perf.HasPerfLogging
import xs.utils.tl.MemReqSource

class SinkA(implicit p: Parameters) extends L2Module with HasPerfLogging{
  val io = IO(new Bundle() {
    val a = Flipped(DecoupledIO(new TLBundleA(edgeIn.bundle)))
    val prefetchReq = prefetchOpt.map(_ => Flipped(DecoupledIO(new PrefetchReq)))
    val task = DecoupledIO(new TaskBundle)
  })
  if(cacheParams.enableAssert) assert(!(io.a.valid && io.a.bits.opcode(2, 1) === 0.U), "no Put")

  val commonReq = Wire(io.task.cloneType)
  val prefetchReq = prefetchOpt.map(_ => Wire(io.task.cloneType))

  io.a.ready := commonReq.ready

  def fromTLAtoTaskBundle(a: TLBundleA): TaskBundle = {
    val task = Wire(new TaskBundle)
    task.channel := "b001".U
    task.tag := parseAddress(a.address)._1
    task.set := parseAddress(a.address)._2
    task.off := parseAddress(a.address)._3
    task.alias.foreach(_ := a.data(aliasBitsOpt.getOrElse(1), 1))
    task.opcode := a.opcode
    task.param := a.param
    task.size := a.size
    task.sourceId := a.source
    task.bufIdx := 0.U(bufIdxBits.W)
    task.needProbeAckData := false.B
    task.mshrTask := false.B
    task.mshrId := 0.U(mshrBits.W)
    task.aliasTask.foreach(_ := false.B)
    task.useProbeData := false.B
    task.pfVec.foreach(_ := PfSource.NONE)
    task.needHint.foreach(_ := a.data(0))
    task.dirty := false.B
    task.way := 0.U(wayBits.W)
    task.meta := 0.U.asTypeOf(new MetaEntry)
    task.metaWen := false.B
    task.tagWen := false.B
    task.dsWen := false.B
    task.wayMask := 0.U(cacheParams.ways.W)
    task.reqSource := a.user.lift(xs.utils.tl.ReqSourceKey).getOrElse(MemReqSource.NoWhere.id.U)
    task.replTask := false.B
    task.vaddr.foreach(_ := a.user.lift(VaddrKey).getOrElse(0.U))
    task.mergeTask := false.B
    task.reqSource := DontCare
    task
  }
  def fromPrefetchReqtoTaskBundle(req: PrefetchReq): TaskBundle = {
    val task = Wire(new TaskBundle)
    val fullAddr = Cat(req.tag, req.set, 0.U(offsetBits.W))
    task.channel := "b001".U
    task.tag := parseAddress(fullAddr)._1
    task.set := parseAddress(fullAddr)._2
    task.off := 0.U
    task.alias.foreach(_ := 0.U)
    task.opcode := Hint
    task.param := Mux(req.needT, PREFETCH_WRITE, PREFETCH_READ)
    task.size := offsetBits.U
    task.sourceId := req.source
    task.bufIdx := 0.U(bufIdxBits.W)
    task.needProbeAckData := false.B
    task.mshrTask := false.B
    task.mshrId := 0.U(mshrBits.W)
    task.aliasTask.foreach(_ := false.B)
    task.useProbeData := false.B
    task.pfVec.foreach(_ := req.pfVec)
    task.needHint.foreach(_ := false.B)
    task.dirty := false.B
    task.way := 0.U(wayBits.W)
    task.meta := 0.U.asTypeOf(new MetaEntry)
    task.metaWen := false.B
    task.tagWen := false.B
    task.dsWen := false.B
    task.wayMask := 0.U(cacheParams.ways.W)
    // TODO: task.reqSource := MemReqSource.L2Prefetch.id.U
    task.reqSource := MemReqSource.Prefetch2L2Unknown.id.U
    task.replTask := false.B
    task.vaddr.foreach(_ := 0.U)
    task.mergeTask := false.B
    task
  }
  commonReq.valid := io.a.valid
  commonReq.bits := fromTLAtoTaskBundle(io.a.bits)
  if (prefetchOpt.nonEmpty) {
    val pipe = Module(new Pipeline(io.prefetchReq.get.bits.cloneType, 1))
    pipe.io.in <> io.prefetchReq.get
    prefetchReq.get.valid := pipe.io.out.valid
    prefetchReq.get.bits := fromPrefetchReqtoTaskBundle(pipe.io.out.bits)
    pipe.io.out.ready := prefetchReq.get.ready
    fastArb(Seq(commonReq, prefetchReq.get), io.task)
  } else {
    io.task <> commonReq
  }

  // Performance counters
  // num of reqs
  XSPerfAccumulate("sinkA_req", io.task.fire)
  XSPerfAccumulate("sinkA_req", io.task.fire)
  XSPerfAccumulate("sinkA_req_is_free", io.task.ready && !io.task.valid)
  XSPerfAccumulate("sinkA_acquire_req", io.a.fire && io.a.bits.opcode(2, 1) === AcquireBlock(2, 1))
  XSPerfAccumulate("sinkA_acquireblock_req", io.a.fire && io.a.bits.opcode === AcquireBlock)
  XSPerfAccumulate("sinkA_acquireperm_req", io.a.fire && io.a.bits.opcode === AcquirePerm)
  XSPerfAccumulate("sinkA_get_req", io.a.fire && io.a.bits.opcode === Get)
  prefetchOpt.foreach {
    _ =>
        XSPerfAccumulate("sinkA_prefetch_req", io.prefetchReq.get.fire)
        XSPerfAccumulate("sinkA_prefetch_from_l1", io.prefetchReq.get.bits.is_l1pf && io.prefetchReq.get.fire)
        XSPerfAccumulate("sinkA_prefetch_from_l2", io.prefetchReq.get.bits.is_l2pf && io.prefetchReq.get.fire)
        XSPerfAccumulate("sinkA_prefetch_from_bop", io.prefetchReq.get.bits.hasBOP && io.prefetchReq.get.fire)
        XSPerfAccumulate("sinkA_prefetch_from_spp", io.prefetchReq.get.bits.hasSPP && io.prefetchReq.get.fire)
  }

  // cycels stalled by mainpipe
  val stall = io.task.valid && !io.task.ready
  XSPerfAccumulate("sinkA_stall_by_mainpipe", stall)
  XSPerfAccumulate("sinkA_acquire_stall_by_mainpipe", stall &&
    (io.task.bits.opcode === AcquireBlock || io.task.bits.opcode === AcquirePerm))
  XSPerfAccumulate("sinkA_get_stall_by_mainpipe", stall && io.task.bits.opcode === Get)
  XSPerfAccumulate("sinkA_put_stall_by_mainpipe", stall &&
    (io.task.bits.opcode === PutFullData || io.task.bits.opcode === PutPartialData))
  prefetchOpt.foreach { _ => XSPerfAccumulate("sinkA_prefetch_stall_by_mainpipe", stall && io.task.bits.opcode === Hint) }
}