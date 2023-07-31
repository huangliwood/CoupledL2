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

package coupledL2.prefetch

import chisel3._
import chisel3.util._
import utility._
import chipsalliance.rocketchip.config.Parameters
import freechips.rocketchip.tilelink._
import coupledL2._
import coupledL2.utils.{XSPerfAccumulate, XSPerfHistogram}

class PrefetchReq(implicit p: Parameters) extends PrefetchBundle {
  val tag = UInt(fullTagBits.W)
  val set = UInt(setBits.W)
  val needT = Bool()
  val source = UInt(sourceIdBits.W)
  val isBOP = Bool()
  def addr = Cat(tag, set, 0.U(offsetBits.W))
}

class PrefetchResp(implicit p: Parameters) extends PrefetchBundle {
  // val id = UInt(sourceIdBits.W)
  val tag = UInt(fullTagBits.W)
  val set = UInt(setBits.W)
  def addr = Cat(tag, set, 0.U(offsetBits.W))
}

class PrefetchTrain(implicit p: Parameters) extends PrefetchBundle {
  // val addr = UInt(addressBits.W)
  val tag = UInt(fullTagBits.W)
  val set = UInt(setBits.W)
  val needT = Bool()
  val source = UInt(sourceIdBits.W)
  // prefetch only when L2 receives a miss or prefetched hit req
  // val miss = Bool()
  // val prefetched = Bool()
  def addr = Cat(tag, set, 0.U(offsetBits.W))
}

class PrefetchEvict(implicit p: Parameters) extends PrefetchBundle {
  // val id = UInt(sourceIdBits.W)
  val tag = UInt(fullTagBits.W)
  val set = UInt(setBits.W)
  def addr = Cat(tag, set, 0.U(offsetBits.W))
}

class PrefetchIO(implicit p: Parameters) extends PrefetchBundle {
  val train = Flipped(DecoupledIO(new PrefetchTrain))
  val req = DecoupledIO(new PrefetchReq)
  val resp = Flipped(DecoupledIO(new PrefetchResp))
  val recv_addr = Flipped(ValidIO(UInt(64.W)))
  val evict = prefetchOpt.get match {
    case hyper: HyperPrefetchParams => Some(Flipped(DecoupledIO(new PrefetchEvict)))
    case _ => None
  }
}

class PrefetchQueue(implicit p: Parameters) extends PrefetchModule {
  val io = IO(new Bundle {
    val enq = Flipped(DecoupledIO(new PrefetchReq))
    val deq = DecoupledIO(new PrefetchReq)
  })
  /*  Here we implement a queue that
   *  1. is pipelined  2. flows
   *  3. always has the latest reqs, which means the queue is always ready for enq and deserting the eldest ones
   */
  val queue = RegInit(VecInit(Seq.fill(inflightEntries)(0.U.asTypeOf(new PrefetchReq))))
  val valids = RegInit(VecInit(Seq.fill(inflightEntries)(false.B)))
  val idxWidth = log2Up(inflightEntries)
  val head = RegInit(0.U(idxWidth.W))
  val tail = RegInit(0.U(idxWidth.W))
  val empty = head === tail && !valids.last
  val full = head === tail && valids.last

  when(!empty && io.deq.ready) {
    valids(head) := false.B
    head := head + 1.U
  }

  when(io.enq.valid) {
    queue(tail) := io.enq.bits
    valids(tail) := !empty || !io.deq.ready // true.B
    tail := tail + (!empty || !io.deq.ready).asUInt
    when(full && !io.deq.ready) {
      head := head + 1.U
    }
  }

  io.enq.ready := true.B
  io.deq.valid := !empty || io.enq.valid
  io.deq.bits := Mux(empty, io.enq.bits, queue(head))

  // The reqs that are discarded = enq - deq
  XSPerfAccumulate(cacheParams, "prefetch_queue_enq", io.enq.fire())
  XSPerfAccumulate(cacheParams, "prefetch_queue_fromL1_enq", io.enq.fire() && !io.enq.bits.isBOP)
  XSPerfAccumulate(cacheParams, "prefetch_queue_fromL2_enq", io.enq.fire() && io.enq.bits.isBOP)
  XSPerfAccumulate(cacheParams, "prefetch_queue_deq", io.deq.fire())
  XSPerfAccumulate(cacheParams, "prefetch_queue_fromL1_deq", io.deq.fire() && !io.enq.bits.isBOP)
  XSPerfAccumulate(cacheParams, "prefetch_queue_fromL2_enq", io.deq.fire() && io.enq.bits.isBOP)
  XSPerfHistogram(cacheParams, "prefetch_queue_entry", PopCount(valids.asUInt),
    true.B, 0, inflightEntries, 1)
}

class Prefetcher(implicit p: Parameters) extends PrefetchModule {
  val io = IO(new PrefetchIO)
  val io_l2_pf_en = IO(Input(Bool()))

  prefetchOpt.get match {
    case spp: SPPParameters => // case spp only
      val pft = Module(new SignaturePathPrefetch)
      val pftQueue = Module(new PrefetchQueue)
      val pipe = Module(new Pipeline(io.req.bits.cloneType, 1))
      pft.io.train <> io.train
      pft.io.resp <> io.resp
      pftQueue.io.enq <> pft.io.req
      pipe.io.in <> pftQueue.io.deq
      io.req <> pipe.io.out
    case bop: BOPParameters => // case bop only
      val pft = Module(new BestOffsetPrefetch)
      val pftQueue = Module(new PrefetchQueue)
      val pipe = Module(new Pipeline(io.req.bits.cloneType, 1))
      pft.io.train <> io.train
      pft.io.resp <> io.resp
      pftQueue.io.enq <> pft.io.req
      pipe.io.in <> pftQueue.io.deq
      io.req <> pipe.io.out
    case receiver: PrefetchReceiverParams => // case sms+bop 
      val l1_pf = Module(new PrefetchReceiver())
      val bop = Module(new BestOffsetPrefetch()(p.alterPartial({
        case L2ParamKey => p(L2ParamKey).copy(prefetch = Some(BOPParameters()))
      })))
      val pftQueue = Module(new PrefetchQueue)
      val pipe = Module(new Pipeline(io.req.bits.cloneType, 1))
      val bop_en = RegNextN(io_l2_pf_en, 2, Some(true.B))
      // l1 prefetch
      l1_pf.io.recv_addr := ValidIODelay(io.recv_addr, 2)
      l1_pf.io.train <> DontCare
      l1_pf.io.resp <> DontCare
      // l2 prefetch
      bop.io.train <> io.train
      bop.io.resp <> io.resp
      // send to prq
      pftQueue.io.enq.valid := l1_pf.io.req.valid || (bop_en && bop.io.req.valid)
      pftQueue.io.enq.bits := Mux(l1_pf.io.req.valid,
        l1_pf.io.req.bits,
        bop.io.req.bits
      )
      l1_pf.io.req.ready := true.B
      bop.io.req.ready := true.B
      pipe.io.in <> pftQueue.io.deq
      io.req <> pipe.io.out
      XSPerfAccumulate(cacheParams, "prefetch_req_fromL1", l1_pf.io.req.valid)
      XSPerfAccumulate(cacheParams, "prefetch_req_fromL2", bop_en && bop.io.req.valid)
      XSPerfAccumulate(cacheParams, "prefetch_req_L1L2_overlapped", l1_pf.io.req.valid && bop_en && bop.io.req.valid)
    
    case hyperPf: HyperPrefetchParams => // case spp +  bop + smsReceiver
      val hybrid_pfts = Module(new HyperPrefetcher())
      val pftQueue = Module(new PrefetchQueue)
      val pipe = Module(new Pipeline(io.req.bits.cloneType, 1))
      hybrid_pfts.io.train <> io.train
      hybrid_pfts.io.resp <> io.resp
      hybrid_pfts.io.recv_addr := ValidIODelay(io.recv_addr, 2)
      io.evict match {
        case Some(evict) =>
        hybrid_pfts.io.evict <> evict
        pftQueue.io.enq <> hybrid_pfts.io.req
        pipe.io.in <> pftQueue.io.deq
        io.req <> pipe.io.out
        case None =>
        hybrid_pfts.io.evict := DontCare
      }
    case fake: FakePrefetchPrarameters =>
      val fakePF = Module(new FakePrefetch())
      val l1_pf = Module(new PrefetchReceiver())
      val pftQueue = Module(new PrefetchQueue)
      val pipe = Module(new Pipeline(io.req.bits.cloneType, 1))
      val l2pf_en = RegNextN(io_l2_pf_en, 2, Some(true.B))
      // l1 prefetch
      l1_pf.io.recv_addr := ValidIODelay(io.recv_addr, 2)
      l1_pf.io.train <> DontCare
      l1_pf.io.resp <> DontCare
      // l2 prefetch
      fakePF.io.clock := this.clock
      fakePF.io.reset := this.reset
      fakePF.io.pf.recv_addr <> io.recv_addr
      fakePF.io.pf.train <> io.train
      fakePF.io.pf.resp <> io.resp
      l2pf_en := true.B
      pftQueue.io.enq.valid := l1_pf.io.req.valid || (l2pf_en && fakePF.io.pf.req.valid)
      pftQueue.io.enq.bits := Mux(l1_pf.io.req.valid,
        l1_pf.io.req.bits,
        fakePF.io.pf.req.bits
      )
      l1_pf.io.req.ready := true.B
      fakePF.io.pf.req.ready := true.B
      pipe.io.in <> pftQueue.io.deq
      io.req <> pipe.io.out
      XSPerfAccumulate(cacheParams, "prefetch_req_fromL1", l1_pf.io.req.valid)
      XSPerfAccumulate(cacheParams, "prefetch_req_fromL2", l2pf_en && pftQueue.io.deq.valid)
      XSPerfAccumulate(cacheParams, "prefetch_req_L1L2_overlapped", l1_pf.io.req.valid && l2pf_en && fakePF.io.pf.req.valid)
    case _ => assert(cond = false, "Unknown prefetcher")
  }
}
