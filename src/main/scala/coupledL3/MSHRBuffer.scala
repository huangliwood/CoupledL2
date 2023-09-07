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

// MSHR Buffer is used when MSHR needs to save data, so each buffer entry corresponds to an MSHR
class MSHRBuffer(implicit p: Parameters) extends L3Module {
  val io = IO(new Bundle() {
    val r = new MSHRBufRead()
    val w = Flipped(DecoupledIO(new MSHRBufWrite))
  })

  val buffer = Module(new SRAMTemplate(new DSBeat(), mshrsAll, beatSize, singlePort = true, hasClkGate = enableClockGate))
  val bufferRead = WireInit(0.U.asTypeOf(new DSBlock()))
  buffer.io.w(
    io.w.fire,
    io.w.bits.data.asTypeOf(Vec(beatSize, new DSBeat())),
    io.w.bits.id,
    io.w.bits.beat_sel
  )
  bufferRead := buffer.io.r(io.r.req.fire, io.r.req.bits.id).resp.data.asTypeOf(new DSBlock)
  io.r.resp.bits.data := bufferRead.data
  io.r.resp.valid := RegNext(io.r.req.fire)

  val valids = RegInit(VecInit.tabulate(mshrsAll, beatSize)((_, _) => false.B))
  val corrupts = RegInit(VecInit.tabulate(mshrsAll, beatSize)((_, _) => false.B))


  when (io.w.valid) {
    valids(io.w.bits.id)(OHToUInt(io.w.bits.beat_sel)) := true.B
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

  io.r.req.ready := !io.w.valid
  io.w.ready := io.w.valid
  dontTouch(io.w.ready)

  val ridReg = RegNext(io.r.req.bits.id, 0.U.asTypeOf(io.r.req.bits.id))

  if(dataEccEnable) {
    io.r.resp.bits.corrupt := VecInit( corrupts.map( entry => entry.reduce(_ || _) ) )(ridReg)
  } else {
    io.r.resp.bits.corrupt := false.B
  }

}

class MSHRBuffer_1(readAlwaysReady: Boolean = false)(implicit p: Parameters) extends L3Module {
  val io = IO(new Bundle() {
    val r = new MSHRBufRead()
    val w = Flipped(DecoupledIO(new MSHRBufWrite))
  })

  val buffer = Module(new SRAMTemplate(new DSBeat(), mshrsAll, beatSize, singlePort = true, hasClkGate = enableClockGate))
  val bufferRead = WireInit(0.U.asTypeOf(new DSBlock()))

  if(readAlwaysReady) {
    io.r.req.ready := true.B
    io.w.ready := !io.r.req.valid
  } else {
    io.r.req.ready := !io.w.valid
    io.w.ready := true.B
  }
  dontTouch(io.w.ready)

  buffer.io.w(
    io.w.fire,
    io.w.bits.data.asTypeOf(Vec(beatSize, new DSBeat())),
    io.w.bits.id,
    io.w.bits.beat_sel
  )
  bufferRead := buffer.io.r(io.r.req.fire, io.r.req.bits.id).resp.data.asTypeOf(new DSBlock)
  io.r.resp.bits.data := bufferRead.data
  io.r.resp.valid := RegNext(io.r.req.fire)

  
  if(dataEccEnable) {
    val corrupts = RegInit(VecInit.tabulate(mshrsAll, beatSize)((_, _) => false.B))

    corrupts.zipWithIndex.foreach{ case(beats, i) => 
      require(beats.getWidth == beatSize, s"${beats.getWidth} =/= ${beatSize}")
      
      when(io.w.fire && io.w.bits.id === i.U) {
        beats.asTypeOf(Vec(beatSize, Bool())).foreach( _ := io.w.bits.corrupt )
      }.elsewhen(io.r.req.fire && io.r.req.bits.id === i.U) {
        beats.asTypeOf(Vec(beatSize, Bool())).foreach( _ := false.B )
      }
    }

    val ridReg = RegNext(io.r.req.bits.id, 0.U.asTypeOf(io.r.req.bits.id))
    io.r.resp.bits.corrupt := VecInit( corrupts.map( entry => entry.reduce(_ || _) ) )(ridReg)
  } else {
    io.r.resp.bits.corrupt := false.B
  }
}

// class MSHRBuffer(implicit p: Parameters) extends L3Module {
//   val io = IO(new Bundle() {
//     val r = new MSHRBufRead()
//     val w = Flipped(DecoupledIO(new MSHRBufWrite))
//   })

//   // Instead of a SRAMTemplate, we now declare a register array.
//   val buffer = RegInit(VecInit(Seq.fill(mshrsAll)(VecInit(Seq.fill(beatSize)(0.U.asTypeOf(new DSBeat()))))))

//   // We now read from and write to the register array directly.
//   when (io.w.fire) {
//     when(io.w.bits.beat_sel === Fill(beatSize, 1.U)) {
//       buffer(io.w.bits.id) := io.w.bits.data.asTypeOf(Vec(beatSize, new DSBeat()))
//     }.otherwise{
//       buffer(io.w.bits.id)(OHToUInt(io.w.bits.beat_sel)) := io.w.bits.data.data.asTypeOf(Vec(beatSize, new DSBeat))(OHToUInt(io.w.bits.beat_sel))
//     }
//   }
//   io.r.resp.valid := RegNext(io.r.req.fire)

//   val valids = RegInit(VecInit.tabulate(mshrsAll, beatSize)((_, _) => false.B))
//   val corrupts = RegInit(VecInit.tabulate(mshrsAll, beatSize)((_, _) => false.B))

//   when (io.w.valid) {
//     valids(io.w.bits.id)(OHToUInt(io.w.bits.beat_sel)) := true.B
//   }

//   when (io.r.req.valid) {
//     valids(io.r.req.bits.id).foreach(_ := false.B)
//     if(dataEccEnable) {
//       corrupts.zipWithIndex.foreach{ case(beats, i) => 
//         require(beats.getWidth == beatSize, s"${beats.getWidth} =/= ${beatSize}")
        
//         when(io.r.req.bits.id === i.U) {
//           beats.asTypeOf(Vec(beatSize, Bool())).foreach( _ := false.B ) 
//         }
//       }
//     }
//   }

//   io.r.req.ready := true.B
//   io.w.ready := true.B
//   dontTouch(io.w.ready)

//   val ridReg = RegNext(io.r.req.bits.id, 0.U.asTypeOf(io.r.req.bits.id))
//   val bufferRead = buffer(ridReg)
//   io.r.resp.bits.data := bufferRead.asTypeOf(io.r.resp.bits.data.cloneType)
//   assert(!(io.w.fire && io.r.req.fire && io.w.bits.id === io.r.req.bits.id))

//   if(dataEccEnable) {
//     io.r.resp.bits.corrupt := VecInit( corrupts.map( entry => entry.reduce(_ || _) ) )(ridReg)
//   } else {
//     io.r.resp.bits.corrupt := false.B
//   }
// }