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
import freechips.rocketchip.tilelink._
import freechips.rocketchip.tilelink.TLMessages._
import org.chipsalliance.cde.config.Parameters

class AcquireUnit(implicit p: Parameters) extends L2Module {
  val io = IO(new Bundle() {
    val sourceA = DecoupledIO(new TLBundleA(edgeOut.bundle))
    val task = Flipped(DecoupledIO(new SourceAReq))
  })

  val a = io.sourceA
  val task = io.task.bits

  a.bits.opcode := task.opcode
  a.bits.param := task.param
  a.bits.size := offsetBits.U
  a.bits.source := task.source
  a.bits.address := Cat(task.tag, task.set, 0.U(offsetBits.W))
  a.bits.mask := Fill(edgeOut.manager.beatBytes, 1.U(1.W))
  a.bits.data := DontCare
  a.bits.echo.lift(huancun.DirtyKey).foreach(_ := true.B)
  a.bits.user.lift(huancun.PreferCacheKey).foreach(_ := false.B)
  a.bits.user.lift(xs.utils.tl.ReqSourceKey).foreach(_ := task.reqSource)
  a.bits.corrupt := false.B

  a.valid := io.task.valid
  io.task.ready := a.ready

  if(cacheParams.enableAssert) {
    val STALL_CNT_MAX = 50000
    val stallCnt = RegInit(0.U(64.W))
    val a = io.sourceA.bits

    when(io.sourceA.valid && !io.sourceA.ready) {
      stallCnt := stallCnt + 1.U
    }.otherwise{
      stallCnt := 0.U
    }

    assert(stallCnt <= STALL_CNT_MAX.U, "sourceA timeout! cnt:%d addr:0x%x sourceId: %d/0x%x", stallCnt, a.address, a.source, a.source)
  }

  dontTouch(io)
}
