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
import java.util.ResourceBundle
import xs.utils.sram._

// read with block granularity
class MSHRBufReadReq(implicit p: Parameters) extends L3Bundle {
  val id = UInt(mshrBits.W)
}

class MSHRBufReadResp(implicit p: Parameters) extends DSBlock {
    val corrupt = Bool()
}

class MSHRBufRead(implicit p: Parameters) extends L3Bundle {
  val req  = Flipped(DecoupledIO(new MSHRBufReadReq))
  val resp = ValidIO(new MSHRBufReadResp)
}


// write with beat granularity
class MSHRBufWrite(implicit p: Parameters) extends L3Bundle {
  val beat_sel = UInt(beatSize.W)
  val data     = new DSBlock
  val id       = UInt(mshrBits.W)
  val corrupt  = Bool()
}

// TODO: should it have both r/w port?
// MSHR Buffer is used when MSHR needs to save data, so each buffer entry corresponds to an MSHR
class MSHRBuffer(wPorts: Int = 1)(implicit p: Parameters) extends L3Module {
  val io = IO(new Bundle() {
    val r = new MSHRBufRead()
    val w = Vec(wPorts, Flipped(DecoupledIO(new MSHRBufWrite)))
  })

  val buffer = RegInit(VecInit.tabulate(mshrsAll, beatSize)((_, _) => 0.U.asTypeOf(new DSBeat())))
  val valids = RegInit(VecInit.tabulate(mshrsAll, beatSize)((_, _) => false.B))
  val corrupts = RegInit(VecInit.tabulate(mshrsAll, beatSize)((_, _) => false.B))

  io.w.foreach {
    case w =>
      when (w.valid) {
        w.bits.beat_sel.asBools.zipWithIndex.foreach {
          case (sel, i) =>
            when (sel) { valids(w.bits.id)(i) := true.B }
        }
      }
  }

  when (io.r.req.valid) {
    valids(io.r.req.bits.id).foreach(_ := false.B)
    if(dataEccEnable) {
      corrupts.zipWithIndex.foreach{ case(beats, i) => 
        require(beats.getWidth == beatSize, s"${beats.getWidth} =/= ${beatSize}")
        
        when(io.r.req.bits.id === i.U) {
          beats.asTypeOf(Vec(beatSize, Bool())).foreach( _ := false.B ) 
        }
      }
    }
  }

  buffer.zipWithIndex.foreach {
    case (block, i) =>
      val wens = VecInit(io.w.map(w => w.valid && w.bits.id === i.U)).asUInt
      assert(PopCount(wens) <= 1.U, "multiple write to the same MSHR buffer entry wens:0x%x", wens)

      val w_beat_sel = PriorityMux(wens, io.w.map(_.bits.beat_sel))
      val w_data = PriorityMux(wens, io.w.map(_.bits.data))
      val w_corrupt = PriorityMux(wens, io.w.map(_.bits.corrupt))
      block.zipWithIndex.foreach {
        case (entry, j) =>
          when(wens.orR && w_beat_sel(j)) {
            entry := w_data.data((j + 1) * beatBytes * 8 - 1, j * beatBytes * 8).asTypeOf(new DSBeat)
          }

          if(dataEccEnable) corrupts(i)(j) := w_corrupt
      }
  }

  io.r.req.ready := true.B
  io.w.foreach(_.ready := true.B)

  val ridReg = RegNext(io.r.req.bits.id, 0.U.asTypeOf(io.r.req.bits.id))
  io.r.resp.bits.data := buffer(ridReg).asUInt
  io.r.resp.valid := true.B // TODO:

  if(dataEccEnable) {
    io.r.resp.bits.corrupt := VecInit( corrupts.map( entry => entry.reduce(_ || _) ) )(ridReg)
  } else {
    io.r.resp.bits.corrupt := false.B
  }

}
