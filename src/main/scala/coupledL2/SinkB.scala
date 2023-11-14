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
import chisel3.util.{DecoupledIO, _}
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.tilelink._
import xs.utils.perf.HasPerfLogging
import xs.utils.tl.MemReqSource

class BMergeTask(implicit p: Parameters) extends L2Bundle {
  val id = UInt(mshrBits.W)
  val task = new TaskBundle()
}

class SinkB(implicit p: Parameters) extends L2Module with HasPerfLogging{
  val io = IO(new Bundle() {
    val b = Flipped(DecoupledIO(new TLBundleB(edgeIn.bundle)))
    val task = DecoupledIO(new TaskBundle)
    val msInfo = Vec(mshrsAll, Flipped(ValidIO(new MSHRInfo)))
    val s3Info = Flipped(ValidIO(new Bundle() {
      val tag = UInt(tagBits.W)
      val set = UInt(setBits.W)
      val willAllocMshr = Bool()
    }))
    val bMergeTask = ValidIO(new BMergeTask)
  })

  def fromTLBtoTaskBundle(b: TLBundleB): TaskBundle = {
    val task = Wire(new TaskBundle)
    task.channel := "b010".U
    task.tag := parseAddress(b.address)._1
    task.set := parseAddress(b.address)._2
    task.off := parseAddress(b.address)._3
    task.alias.foreach(_ := 0.U)
    task.vaddr.foreach(_ := 0.U)
    task.opcode := b.opcode
    task.param := b.param
    task.size := b.size
    task.sourceId := 0.U(sourceIdBits.W)
    task.bufIdx := 0.U(bufIdxBits.W)
//    task.needProbeAckData := b.data(0) // TODO: parameterize this
    task.needProbeAckData := true.B // TODO: parameterize this
    task.mshrTask := false.B
    task.mshrId := 0.U(mshrBits.W)
    task.aliasTask.foreach(_ := false.B)
    task.useProbeData := false.B
    task.fromL2pft.foreach(_ := false.B)
    task.needHint.foreach(_ := false.B)
    task.dirty := false.B
    task.way := 0.U(wayBits.W)
    task.meta := 0.U.asTypeOf(new MetaEntry)
    task.metaWen := false.B
    task.tagWen := false.B
    task.dsWen := false.B
    task.wayMask := Fill(cacheParams.ways, "b1".U)
    task.reqSource := MemReqSource.NoWhere.id.U // Ignore
    task.replTask := false.B
    task.mergeTask := false.B
    task
  }
  val task = fromTLBtoTaskBundle(io.b.bits)

  /* ======== Merge Nested-B req ======== */
  // unable to accept incoming B req because same-addr as some MSHR REQ
  val addrConflict = VecInit(io.msInfo.map(s =>
    s.valid && s.bits.set === task.set && s.bits.reqTag === task.tag && !s.bits.willFree && !s.bits.nestB
  )).asUInt.orR

  // unable to accept incoming B req because same-addr as some MSHR replaced block and cannot nest
  val replaceConflictMask = VecInit(io.msInfo.map(s =>
    s.valid && s.bits.set === task.set && s.bits.metaTag === task.tag && s.bits.releaseNotSent && !s.bits.mergeB
  )).asUInt
  val replaceConflict = replaceConflictMask.orR

  // incoming B can be merged with some MSHR replaced block and able to be accepted
  val mergeBMask = VecInit(io.msInfo.map(s =>
    s.valid && s.bits.set === task.set && s.bits.metaTag === task.tag && s.bits.mergeB
  )).asUInt

  // unable to accept incoming B req because same-addr as mainpipe S3
  val s3AddrConflict = io.s3Info.valid && io.s3Info.bits.set === task.set && io.s3Info.bits.tag === task.tag && io.s3Info.bits.willAllocMshr

  assert(PopCount(replaceConflictMask) <= 1.U)
  assert(PopCount(mergeBMask) <= 1.U)

  val mergeB = mergeBMask.orR // TODO: && task.param === toN // only toN can merge with MSHR-Release
  val mergeBId = OHToUInt(mergeBMask)

  // when conflict, we block B req from entering SinkB
  // when !conflict and mergeB , we merge B req to MSHR
  val task_temp = WireInit(0.U.asTypeOf(io.task))
  task_temp.valid := io.b.valid && !addrConflict && !replaceConflict && !mergeB && !s3AddrConflict
  task_temp.bits := task
  io.b.ready :=  mergeB || (task_temp.ready && !addrConflict && !replaceConflict && !s3AddrConflict)
  val taskOutPipe = Queue(task_temp, entries = 1, pipe = true, flow = false) // for timing: mshrCtl <> sinkB <> reqArb
  io.task <> taskOutPipe

  val bMergeTask = Wire(Decoupled(new BMergeTask))
  bMergeTask.valid := io.b.valid && mergeB && !s3AddrConflict
  bMergeTask.bits.id := mergeBId
  bMergeTask.bits.task := task
  val bMergeTaskOutPipe = Queue(bMergeTask, entries = 1, pipe = true, flow = false) // for timing: mshrCtl <> sinkB <> mshrCtl
  io.bMergeTask.valid := bMergeTaskOutPipe.valid
  io.bMergeTask.bits := bMergeTaskOutPipe.bits
  bMergeTaskOutPipe.ready := true.B

  //--------------------------------- assert ----------------------------------------//
  // unable to accept incoming B req because same-addr as some MSHR REQ
  val addrConflict_s = VecInit(io.msInfo.map(s =>
    s.valid && s.bits.set === io.task.bits.set && s.bits.reqTag === io.task.bits.tag && !s.bits.willFree && !s.bits.nestB
  )).asUInt.orR

  // unable to accept incoming B req because same-addr as some MSHR replaced block and cannot nest
  val replaceConflictMask_s = VecInit(io.msInfo.map(s =>
    s.valid && s.bits.set === io.task.bits.set && s.bits.metaTag === io.task.bits.tag && s.bits.releaseNotSent && !s.bits.mergeB
  )).asUInt
  val replaceConflict_s = replaceConflictMask_s.orR

  // incoming B can be merged with some MSHR replaced block and able to be accepted
  val mergeBMask_s = VecInit(io.msInfo.map(s =>
    s.valid && s.bits.set === io.task.bits.set && s.bits.metaTag === io.task.bits.tag && s.bits.mergeB
  )).asUInt

  val mergeB_s = mergeBMask_s.orR

  val io_task_can_valid = !addrConflict_s && !replaceConflict_s && !mergeB_s
  val io_bMergeTask_can_valid = mergeB_s
  when(io.task.valid){
    assert(io_task_can_valid, "io_task cant valid")
  }
  when(io.bMergeTask.valid){
    assert(io_bMergeTask_can_valid, "io_bMergeTask cant valid")
  }



  XSPerfAccumulate("mergeBTask", io.bMergeTask.valid)
  XSPerfAccumulate("mp_s3_block_sinkB", io.b.valid && !addrConflict && !replaceConflict && !mergeB && s3AddrConflict)
  //!!WARNING: TODO: if this is zero, that means fucntion [Probe merge into MSHR-Release] is never tested, and may have flaws
}
