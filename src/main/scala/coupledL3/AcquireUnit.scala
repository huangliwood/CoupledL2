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

package coupledL3

import chisel3._
import chisel3.util._
import xs.utils._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.tilelink.TLMessages._
import org.chipsalliance.cde.config.Parameters
import huancun.{PreferCacheKey}

class AcquireUnit(implicit p: Parameters) extends L3Module {
  val io = IO(new Bundle() {
    val sourceA = DecoupledIO(new TLBundleA(edgeOut.bundle))
    val task = Flipped(DecoupledIO(new SourceAReq))
  })

  val a = io.sourceA
  val a_acquire = Wire(a.cloneType)
  val task = io.task.bits

  a_acquire.valid := io.task.valid
  a_acquire.bits.opcode := task.opcode
  a_acquire.bits.param := task.param
  a_acquire.bits.size := offsetBits.U
  a_acquire.bits.source := task.source
  a_acquire.bits.address := Cat(task.tag, task.set, 0.U(offsetBits.W))
  a_acquire.bits.mask := Fill(edgeOut.manager.beatBytes, 1.U(1.W))
  a_acquire.bits.data := DontCare
  a_acquire.bits.echo.lift(DirtyKey).foreach(_ := true.B)
  a_acquire.bits.user.lift(PreferCacheKey).foreach(_ := false.B)
  a_acquire.bits.user.lift(xs.utils.tl.ReqSourceKey).foreach(_ := task.reqSource)
  a_acquire.bits.corrupt := false.B

  io.sourceA <> a_acquire
  io.sourceA.valid := a_acquire.valid

  io.task.ready := a_acquire.ready

  dontTouch(io)
}