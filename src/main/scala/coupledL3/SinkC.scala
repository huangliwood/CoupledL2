/** *************************************************************************************
 * Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
 * Copyright (c) 2020-2021 Peng Cheng Laboratory
 *
 * XiangShan is licensed under Mulan PSL v2.
 * You can use this software according to the terms and conditions of the Mulan PSL v2.
 * You may obtain a copy of Mulan PSL v2 at:
 *          http://license.coscl.org.cn/MulanPSL2
 *
 * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
 * EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
 * MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
 *
 * See the Mulan PSL v2 for more details.
 * *************************************************************************************
 */

package coupledL3

import chisel3._
import chisel3.util._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.tilelink.TLMessages._
import chipsalliance.rocketchip.config.Parameters
import coupledL3.utils.XSPerfAccumulate

class PipeBufferRead(implicit p: Parameters) extends L3Bundle {
  val bufIdx = UInt(bufIdxBits.W)
}

class PipeBufferResp(implicit p: Parameters) extends L3Bundle {
  val data = Vec(beatSize, UInt((beatBytes * 8).W))
}

// SinkC receives upwards Release or ProbeAck:
// (1) For Release/ReleaseData, send it to RequestArb directly
// (2) For ProbeAck/ProbeAckData, wakeup w_probeack in MSHR
//     For ProbeAckData, save data into ReleaseBuffer
class SinkC(implicit p: Parameters) extends L3Module {
  val io = IO(new Bundle() {
    val c = Flipped(DecoupledIO(new TLBundleC(edgeIn.bundle)))
    val toReqArb = DecoupledIO(new TaskBundle) // Release/ReleaseData
    val resp = Output(new RespBundle)
    val releaseBufWrite = Flipped(new MSHRBufWrite)
    val bufRead = Input(ValidIO(new PipeBufferRead))
    val bufResp = Output(new PipeBufferResp)
    val mshrStatus  = Vec(mshrsAll, Flipped(ValidIO(new MSHRBlockAInfo)))
    val mshrFull = Input(Bool())
  })

  val (first, last, _, beat) = edgeIn.count(io.c)
  val isRelease = io.c.bits.opcode(1)
  val hasData = io.c.bits.opcode(0)

  // dataBuf entry is valid when Release has data
  val dataBuf = Reg(Vec(bufBlocks, Vec(beatSize, UInt((beatBytes * 8).W))))
  val beatValids = RegInit(VecInit(Seq.fill(bufBlocks)(VecInit(Seq.fill(beatSize)(false.B)))))
  val dataValids = VecInit(beatValids.map(_.asUInt.orR)).asUInt
  val bufValids = dataValids

  val full = bufValids.andR
  val noSpace = full && hasData
  val nextPtr = PriorityEncoder(~bufValids)
  val nextPtrReg = RegEnable(nextPtr, 0.U.asTypeOf(nextPtr), io.c.fire() && isRelease && first && hasData)
  val tag = parseAddress(io.c.bits.address)._1
  val set = parseAddress(io.c.bits.address)._2

  def toTaskBundle(c: TLBundleC): TaskBundle = {
    val task = Wire(new TaskBundle)
    task := DontCare
    task.channel := "b100".U
    task.tag := parseAddress(c.address)._1
    task.set := parseAddress(c.address)._2
    task.off := parseAddress(c.address)._3
    task.alias.foreach(_ := 0.U)
    task.opcode := c.opcode
    task.param := c.param
    task.size := c.size
    task.sourceId := c.source
    task.mshrTask := false.B
    task.wayMask := Fill(cacheParams.ways, "b1".U)
    task.reqSource := MemReqSource.NoWhere.id.U // Ignore
    task
  }

  when (io.c.fire() && isRelease) {
    when (hasData) {
      when (first) {
        dataBuf(nextPtr)(beat) := io.c.bits.data
        beatValids(nextPtr)(beat) := true.B
      }.otherwise {
        assert(last)
        dataBuf(nextPtrReg)(beat) := io.c.bits.data
        beatValids(nextPtrReg)(beat) := true.B
      }
    }
  }

  when (io.bufRead.valid) {
    beatValids(io.bufRead.bits.bufIdx).foreach(_ := false.B)
  }


  io.toReqArb.valid := io.c.valid && first && isRelease && !noSpace
  io.toReqArb.bits := toTaskBundle(io.c.bits)
  io.toReqArb.bits.bufIdx := nextPtr

  io.resp.valid := io.c.valid && (first || last) && !isRelease
  io.resp.mshrId := 0.U // DontCare
  io.resp.tag := tag
  io.resp.set := set
  io.resp.respInfo := DontCare
  io.resp.respInfo.opcode := io.c.bits.opcode
  io.resp.respInfo.param := io.c.bits.param
  io.resp.respInfo.last := last
  io.resp.respInfo.dirty := io.c.bits.opcode(0)
  io.resp.respInfo.source := io.c.bits.source

  io.releaseBufWrite.valid := io.c.valid && io.c.bits.opcode === ProbeAckData
  io.releaseBufWrite.beat_sel := UIntToOH(beat)
  io.releaseBufWrite.data.data := Fill(beatSize, io.c.bits.data)
  io.releaseBufWrite.id := DontCare // id is given by MSHRCtl by comparing address to the MSHRs (only for ProbeAckData)
  io.releaseBufWrite.corrupt := false.B

//  io.c.ready := !isRelease || // ProbeAck / ProbeAckData
//                isRelease && first && !io.mshrFull || // Release / ReleaseData
//                !hasData && io.toReqArb.ready || // TODO: why?
//                !first

  def getMSHRSameSetVec(valid: Bool, set: UInt) = {
    val mshrSameSetVec = VecInit(io.mshrStatus.map { status =>
      status.valid && valid && status.bits.set === set
    })
    mshrSameSetVec
  }
  def getOccWayVec(valid: Bool, set: UInt) = {
    val mshrSameSetVec = getMSHRSameSetVec(valid, set)
    val occWay = mshrSameSetVec.zip(io.mshrStatus).map{
      case(sameSet, status) => 
        Mux(sameSet, UIntToOH(status.bits.way), 0.U(cacheParams.ways.W))
    }.reduce(_|_)
    occWay
  }
  val occWay = getOccWayVec(io.c.valid, set)
  val hasEmptyWay = ~occWay.andR

  io.c.ready := Mux(first, !noSpace && !(isRelease && !io.toReqArb.ready), true.B)

  
  io.toReqArb.bits.wayMask := ~occWay
  when(RegNext(io.c.valid && isRelease && !io.toReqArb.ready && io.mshrFull)) {
    printf("[WARN] MSHR is full release is block!\n")
  }
  
  io.bufResp.data := dataBuf(io.bufRead.bits.bufIdx)

  // Performance counters
  val stall = io.c.valid && isRelease && !io.c.ready
  XSPerfAccumulate(cacheParams, "sinkC_c_stall", stall)
  XSPerfAccumulate(cacheParams, "sinkC_c_stall_for_noSpace", stall && hasData && first && full)
  XSPerfAccumulate(cacheParams, "sinkC_toReqArb_stall", io.toReqArb.valid && !io.toReqArb.ready)
  XSPerfAccumulate(cacheParams, "sinkC_buf_full", full)
}