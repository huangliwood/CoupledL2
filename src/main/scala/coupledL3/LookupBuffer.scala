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
import chipsalliance.rocketchip.config.Parameters
import coupledL3.utils._
import xs.utils.sram._

// read with block granularity
class LookupBufRead(implicit p: Parameters) extends L3Bundle {
  val valid = Input(Bool())
  val id = Input(UInt(lookupBufBits.W))
  val data = Output(new DSBlock)
  val corrupt = Output(Bool())
}

// write with beat granularity
class LookupBufWrite(implicit p: Parameters) extends L3Bundle {
  val valid = Input(Bool())
  val id = Input(UInt(lookupBufBits.W))
  val beat_sel = Input(UInt(beatSize.W))
  val data = Input(new DSBlock)
  val corrupt = Input(Bool())
}


class LookupBuffer(entries: Int = 16)(implicit p: Parameters) extends L3Module {
  val io = IO(new Bundle{
    val r = new LookupBufRead()
    val w = new LookupBufWrite()
    val full = Output(Bool())
  })

  val buffer = RegInit(VecInit.tabulate(mshrsAll, beatSize)((_, _) => 0.U.asTypeOf(new DSBeat())))
  val valids = RegInit(VecInit.tabulate(mshrsAll, beatSize)((_, _) => false.B))
  val corrupts = RegInit(VecInit.tabulate(mshrsAll, beatSize)((_, _) => false.B))

  io.full := Cat(valids.map( e => Cat(e).andR )).andR()


  when(io.w.valid) {
    io.w.beat_sel.asBools.zipWithIndex.foreach {
      case (sel, i) =>
        when(sel) {
          valids(io.w.id)(i) := true.B
        }
    }
  }

  when(io.r.valid) {
    valids(io.r.id).foreach(_ := false.B)

    if (dataEccEnable) {
      corrupts.zipWithIndex.foreach {
        case (beats, i) =>
          require(beats.getWidth == beatSize, s"${beats.getWidth} =/= ${beatSize}")

          when(io.r.id === i.U) {
            beats.asTypeOf(Vec(beatSize, Bool())).foreach(_ := false.B)
          }
      }
    }
  }


  buffer.zipWithIndex.foreach{
    case (buf, i) =>
      buf.zipWithIndex.foreach{
        case (entry, j) =>
          when(io.w.valid && io.w.beat_sel(j)) {
            entry := io.w.data.data((j + 1) * beatBytes * 8 - 1, j * beatBytes * 8).asTypeOf(new DSBeat)
          }
      }
  }


  val ridReg = RegNext(io.r.id, 0.U.asTypeOf(io.r.id))
  io.r.data.data := buffer(ridReg).asUInt

  if (dataEccEnable) {
    io.r.corrupt := VecInit(corrupts.map(entry => entry.reduce(_ || _)))(ridReg)
  } else {
    io.r.corrupt := false.B
  }
}