package coupledL2.prefetch

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import freechips.rocketchip.tilelink._
import coupledL2._
import coupledL2.HasCoupledL2Parameters
import xs.utils.perf.HasPerfLogging
import xs.utils.SRAMQueue
import xs.utils.ParallelPriorityMux
import freechips.rocketchip.diplomacy.BufferParams
import xs.utils.FastArbiter
import javax.management.loading.MLetContent
import freechips.rocketchip.rocket.PRV

case class HyperPrefetchParams(
  fTableEntries: Int = 32,
  pTableQueueEntries: Int = 2
)
    extends PrefetchParameters {
  override val hasPrefetchBit:  Boolean = true
  override val hasPrefetchSrc:  Boolean = true
  override val inflightEntries: Int = 32
}

trait HasHyperPrefetcherParams extends HasCoupledL2Parameters {
  val hyperPrefetchParams = prefetchOpt.get.asInstanceOf[HyperPrefetchParams]
  
  val blkAddrBits = fullAddressBits - offsetBits
  val pageAddrBits = fullAddressBits - pageOffsetBits
  val blkOffsetBits = pageOffsetBits - offsetBits
  val blkNums = 1<<blkOffsetBits //64

  val fTableEntries = hyperPrefetchParams.fTableEntries
  val fTagBits = pageAddrBits - log2Up(fTableEntries)
  val pTableQueueEntries = hyperPrefetchParams.pTableQueueEntries
  val fTableQueueEntries = 128

  def get_blockAddr(x:UInt) = x(fullAddressBits-1,offsetBits)
}

abstract class PrefetchBranchV2Module(implicit val p: Parameters) extends Module with HasHyperPrefetcherParams
abstract class PrefetchBranchV2Bundle(implicit val p: Parameters) extends Bundle with HasHyperPrefetcherParams

// class FilterV2(parentName:String = "Unknown")(implicit p: Parameters) extends PrefetchBranchV2Module with HasPerfLogging{
//   val io = IO(new Bundle() {
//     val req = Flipped(DecoupledIO(new PrefetchReq))
//     val resp = DecoupledIO(new PrefetchReq)
//     val evict = Flipped(DecoupledIO(new PrefetchEvict))
//     val isForce = Input(Bool())
//   })

//   def idx(addr:      UInt) = addr(log2Up(fTableEntries) - 1, 0)
//   def tag(addr:      UInt) = addr(pageAddrBits - 1, log2Up(fTableEntries))

//   def fTableEntry() = new Bundle {
//     val valid = Bool()
//     val tag = UInt(fTagBits.W)
//     val bitMap = Vec(64, Bool())
//   }

//   val fTable = RegInit(VecInit(Seq.fill(fTableEntries)(0.U.asTypeOf(fTableEntry()))))
//   val q = Module(new ReplaceableQueueV2(UInt(fullAddressBits.W), fTableQueueEntries))

//   val oldAddr = io.req.bits.addr
//   val blkAddr = oldAddr(fullAddressBits - 1, offsetBits)
//   val pageAddr = oldAddr(fullAddressBits - 1, pageOffsetBits)
//   val blkOffset = oldAddr(pageOffsetBits - 1, offsetBits)

//   //read fTable
//   val hit = Wire(Bool())
//   val readResult = Wire(fTableEntry())
//   readResult := fTable(idx(pageAddr))
//   hit := readResult.valid && tag(pageAddr) === readResult.tag
//   val hitForMap = hit && readResult.bitMap(blkOffset)

//   io.resp.valid := io.req.fire && (io.req.bits.pfVec =/= PfSource.SPP || !hitForMap)
//   io.resp.bits := io.req.bits

//   val wData = Wire(fTableEntry())
//   val newBitMap = readResult.bitMap.zipWithIndex.map{ case (b, i) => Mux(i.asUInt === blkOffset, true.B, false.B) }
  
//   wData.valid := true.B
//   wData.tag := tag(pageAddr)
//   wData.bitMap := newBitMap
//   when(io.req.fire) {
//     when(hit) {
//       fTable(idx(pageAddr)).bitMap(blkOffset) := true.B
//     } .otherwise {
//       fTable(idx(pageAddr)) := wData
//     }
//   }

//   q.io.enq.valid := io.req.fire && !hitForMap
//   q.io.enq.bits := io.req.bits.addr
//   q.io.deq.ready := q.io.full && q.io.enq.fire

//   val evictAddr = q.io.deq.bits
//   val evictPageAddr = evictAddr(fullAddressBits - 1, pageOffsetBits)
//   val evictBlkOffset = evictAddr(pageOffsetBits - 1, offsetBits)
//   val evictBlkAddr = evictAddr(fullAddressBits - 1, offsetBits)
//   val readEvict = Wire(fTableEntry())
//   val hitEvict = Wire(Bool())
//   val conflict = io.req.fire && blkAddr === evictBlkAddr
//   readEvict := fTable(idx(evictPageAddr))
//   hitEvict := q.io.deq.fire && readEvict.valid && tag(evictPageAddr) === readEvict.tag && readEvict.bitMap(evictBlkOffset) && !conflict
//   when(hitEvict) {
//     fTable(idx(evictPageAddr)).bitMap(evictBlkOffset) := false.B
//   }

//   /*
//   val evictAddr = io.evict.bits.addr
//   val evictPageAddr = evictAddr(fullAddressBits - 1, pageOffsetBits)
//   val evictBlkOffset = evictAddr(pageOffsetBits - 1, offsetBits)
//   val evictBlkAddr = evictAddr(fullAddressBits - 1, offsetBits)
//   val readEvict = Wire(fTableEntry())
//   val hitEvict = Wire(Bool())
//   val conflict = io.req.fire && blkAddr === evictBlkAddr
//   readEvict := fTable(idx(evictPageAddr))
//   hitEvict := io.evict.valid && readEvict.valid && tag(evictPageAddr) === readEvict.tag && readEvict.bitMap(evictBlkOffset) && !conflict
//   when(hitEvict) {
//     fTable(idx(evictPageAddr)).bitMap(evictBlkOffset) := false.B
//   }*/

//   io.req.ready := true.B
//   io.evict.ready := true.B
//   XSPerfAccumulate("hyper_filter_nums",io.req.fire && hitForMap)
//   XSPerfAccumulate("hyper_filter_input",io.req.fire)
//   XSPerfAccumulate("hyper_filter_output",io.resp.fire)
//   XSPerfAccumulate("hyper_filter_bop_req",io.resp.valid && io.resp.bits.pfVec === PfSource.BOP)
//   XSPerfAccumulate("hyper_filter_sms_req",io.resp.valid && io.resp.bits.pfVec === PfSource.SMS)
//   XSPerfAccumulate("hyper_filter_spp_req",io.resp.valid && io.resp.bits.pfVec === PfSource.SPP)
// }

class FilterTable(parentName:String = "Unknown")(implicit p: Parameters) extends PrefetchBranchV2Module with HasPerfLogging{
  val io = IO(new Bundle() {
    val from_pfQ = Flipped(DecoupledIO(new PrefetchReq))
    val evict = Flipped(DecoupledIO(new PrefetchEvict))
    val from_mlfq1 = Flipped(DecoupledIO(new PrefetchReq))
    val qurryResp_mlfq1 = ValidIO(new Bundle {
      val needDrop = Bool()
    })
    val out_mlfq1 = DecoupledIO(new PrefetchReq)
    val out_mlfq2 = DecoupledIO(new PrefetchReq)
    val from_sppHintQ = Flipped(DecoupledIO(new PrefetchReq))
    val out_hintllc = ValidIO(new PrefetchReq)
  })
  def idx(addr:      UInt) = addr(log2Up(fTableEntries) - 1, 0)
  def tag(addr:      UInt) = addr(pageAddrBits - 1, log2Up(fTableEntries))

  object FitlerVecState {
  val bits = 3

  def toN = 0.U(bits.W)
  def toB = 1.U(bits.W)
  def toS = 2.U(bits.W)
  def toC = 3.U(bits.W)

  def None          = 0.U(bits.W)
  def BOP           = 1.U(bits.W)
  def SPP           = 2.U(bits.W)
  def SMS           = 4.U(bits.W)
  def COMMON        = 3.U(bits.W)

  def getVecState(isHit:Bool, originV:UInt, trigerId:UInt) = originV | trigerId
  def checkOne(v:UInt) = v === BOP || v === SPP
  def checkTwo(v:UInt) = v === COMMON
  def hasMyself(v:UInt,originV:UInt) = (v & originV) === v
  def hasMerged(v:UInt,originV:UInt) = v =/= originV
  def is_SPPchase(v:UInt,originV:UInt) = originV === BOP && v === SPP 
}
  val dupNums = 8

  val req_dups = RegInit(VecInit(Seq.fill(dupNums)(0.U.asTypeOf(Valid(new PrefetchReq)))))

  val req_issue = WireInit(0.U.asTypeOf(DecoupledIO(new PrefetchReq())));dontTouch(req_issue)
  dontTouch(io.from_mlfq1)
  fastArb(Seq(io.from_pfQ,io.from_mlfq1,io.from_sppHintQ),req_issue)

  val s1_frompfQ = RegNext(io.from_pfQ.fire)
  val s1_from_mlfq1 = RegNext(io.from_mlfq1.fire)
  val s1_from_sppHintQ = RegNext(io.from_sppHintQ.fire)

  // val req_hint2llc = RegNext(io.from_sppHintQ,false.B)
  req_dups.foreach(x =>  {
    x.valid := req_issue.valid
    x.bits := req_issue.bits
  })
  req_issue.ready := true.B
  val dupOffsetBits = log2Up(fTableEntries/dupNums)
  val dupBits = log2Up(dupNums)
  // --------------------------------------------------------------------------------
  // consensus Table cTable
  // --------------------------------------------------------------------------------
  // | valid | tag | cVec[[pfVec],[pfVec],...,[pfVec]]
  // | valid | tag | cVec[[pfVec],[pfVec],...,[pfVec]]
  //                               ^
  //                               |
  //                            archored_value
  def fTableEntry() = new Bundle {
    val valid = Bool()
    val tag = UInt(fTagBits.W)
    val cVec = Vec(blkNums, UInt(FitlerVecState.bits.W))
  }
  val consensusTable = RegInit(VecInit(Seq.fill(fTableEntries)(0.U.asTypeOf(fTableEntry()))))
  // val consensusTable = Mem(fTableEntries,fTableEntry())
  // val q = Module(new Queue(UInt(fullAddressBits.W), fTableQueueEntries, flow = false, pipe = true)) //TODO: opimize timing
  val evict_q = Module(new SRAMQueue(UInt(blkAddrBits.W),entries = fTableQueueEntries, flow = true, 
    hasMbist = cacheParams.hasMbist, hasClkGate=enableClockGate, hasShareBus = cacheParams.hasShareBus, parentName=parentName+"filterDelayQ"))

  val hit = WireInit(VecInit.fill(dupNums)(false.B))
  val readResult = WireInit(VecInit.fill(dupNums)(0.U.asTypeOf(fTableEntry())));dontTouch(readResult)
  val hitForMap_one = WireInit(VecInit.fill(dupNums)(false.B));dontTouch(hitForMap_one)
  val hitForMap_two = WireInit(VecInit.fill(dupNums)(false.B));dontTouch(hitForMap_two)
  val hitForMap_needDrop = WireInit(VecInit.fill(dupNums)(false.B));dontTouch(hitForMap_needDrop)
  val can_send2_mlfq1 = WireInit(VecInit.fill(dupNums)(false.B))
  val can_send2_mlfq2 = WireInit(VecInit.fill(dupNums)(false.B))
  val next_VecState = WireInit(VecInit.fill(dupNums)(0.U(PfVectorConst.bits.W)))
  
  val wBitMap = WireInit(VecInit.fill(blkNums)(0.U(FitlerVecState.bits.W)));dontTouch(wBitMap)
  val wBitMergeMask = WireInit(VecInit.fill(blkNums)(false.B))
  val wData = WireInit(0.U.asTypeOf(fTableEntry()))
  val dup_offset = req_dups(0).bits.set(dupOffsetBits-1+dupBits,dupOffsetBits-1)
  for(i <- 0 until(dupNums)) {
    when(req_dups(i).bits.set(dupOffsetBits-1+dupBits-1,dupOffsetBits-1) === i.U(dupBits.W)) {
      val oldAddr = req_dups(i).bits.addr
      val pageAddr = oldAddr(fullAddressBits - 1, pageOffsetBits)
      val blkOffset = oldAddr(pageOffsetBits - 1, offsetBits)

      //read cTable
      readResult(i) := consensusTable(idx(pageAddr))
      val trigerId = req_dups(i).bits.pfVec
      hit(i) := readResult(i).valid

      val anchored_cVec = readResult(i).cVec
      val anchored_value = readResult(i).cVec(blkOffset)
      hitForMap_needDrop(i) := hit(i) && FitlerVecState.hasMerged(req_dups(i).bits.pfVec, anchored_value)
      next_VecState(i) := FitlerVecState.getVecState(true.B, originV = anchored_value, trigerId = req_dups(i).bits.pfVec)
      
      for (j <- 0 until readResult(i).cVec.length){
        wBitMap(j) := Mux(j.asUInt === blkOffset, next_VecState(i), anchored_cVec(j))
        wBitMergeMask(j) := Mux(j.asUInt === blkOffset,next_VecState(i) =/= anchored_value, false.B)
      }

      hitForMap_two(i) := hit(i) && FitlerVecState.checkTwo(anchored_value)
      hitForMap_one(i) := hit(i) && FitlerVecState.checkOne(anchored_value)

      // need hit 1 on another one and hit 0 on itself
      can_send2_mlfq2(i) := !FitlerVecState.is_SPPchase(req_dups(i).bits.pfVec,anchored_value) && hitForMap_one(i) && wBitMergeMask.reduce(_ || _)
      can_send2_mlfq1(i) := !hitForMap_one(i)

      wData.valid := true.B
      wData.tag := tag(pageAddr)
      wData.cVec := wBitMap
      when(req_dups(i).valid) {
          consensusTable(idx(pageAddr)) := wData
      }
    }.otherwise{
      readResult(i) := 0.U.asTypeOf(fTableEntry())
    }
  }

  io.qurryResp_mlfq1.valid := s1_from_mlfq1
  io.qurryResp_mlfq1.bits.needDrop := hitForMap_needDrop(dup_offset)

  io.out_mlfq1.valid := s1_frompfQ && req_dups.zip(can_send2_mlfq1).map(x => x._1.valid && x._2).reduce(_ ||_ )
  io.out_mlfq1.bits := req_dups(dup_offset).bits

  io.out_mlfq2.valid := s1_frompfQ && req_dups(dup_offset).valid && (can_send2_mlfq2.reduce(_ || _))
  io.out_mlfq2.bits := req_dups(dup_offset).bits
  io.out_mlfq2.bits.pfVec := next_VecState(dup_offset)

  io.out_hintllc.valid := false.B//req_dups(1) && req_hint2llc
  io.out_hintllc.bits := 0.U.asTypeOf(io.out_hintllc.bits.cloneType) //req_dups(1)

  evict_q.io.enq.valid := req_dups(dup_offset).valid && (!hitForMap_one(dup_offset) && !hitForMap_two(dup_offset)) // if spp2llc , don't enq
  evict_q.io.enq.bits := get_blockAddr(req_dups(dup_offset).bits.addr)

  val isqFull = evict_q.io.count === (fTableQueueEntries-1).U
  evict_q.io.deq.ready := false.B//isqFull;dontTouch(evict_q.io.deq.ready)

  // --------------------------------------------------------------------------------
  // evict operation
  // --------------------------------------------------------------------------------
  val evictAddr = WireInit(Cat(evict_q.io.deq.bits,0.U(offsetBits.W)))
  val evictPageAddr = evictAddr(fullAddressBits - 1, pageOffsetBits)
  val evictBlkOffset = evictAddr(pageOffsetBits - 1, offsetBits)
  val evictBlkAddr = evictAddr(fullAddressBits - 1, offsetBits)
  val readEvict = WireInit(0.U.asTypeOf(fTableEntry()))
  val hitEvict =  WireInit(false.B)
  val req_evict = req_dups(3)

  val oldAddr = req_evict.bits.addr
  val blkAddr = oldAddr(fullAddressBits - 1, offsetBits)
  val conflict = req_evict.valid && blkAddr === evictBlkAddr
  readEvict := consensusTable(idx(evictPageAddr))
  hitEvict := evict_q.io.deq.fire && readEvict.valid && tag(evictPageAddr) === readEvict.tag && !conflict
  when(hitEvict) {
    consensusTable(idx(evictPageAddr)).cVec(evictBlkOffset) := FitlerVecState.None
  }
  
  /*
  val evictAddr = io.evict.bits.addr
  val evictPageAddr = evictAddr(fullAddressBits - 1, pageOffsetBits)
  val evictBlkOffset = evictAddr(pageOffsetBits - 1, offsetBits)
  val evictBlkAddr = evictAddr(fullAddressBits - 1, offsetBits)
  val readEvict = Wire(fTableEntry())
  val hitEvict = Wire(Bool())
  val conflict = io.req.fire && blkAddr === evictBlkAddr
  readEvict := fTable(idx(evictPageAddr))
  hitEvict := io.evict.valid && readEvict.valid && tag(evictPageAddr) === readEvict.tag && readEvict.bitMap(evictBlkOffset) && !conflict
  when(hitEvict) {
    fTable(idx(evictPageAddr)).bitMap(evictBlkOffset) := false.B
  }*/

  io.from_pfQ.ready := true.B
  io.evict.ready := true.B
  XSPerfAccumulate("hyper_filter_nums",io.from_pfQ.fire && (hitForMap_one(0)||hitForMap_two(0)))
  XSPerfAccumulate("hyper_filter_input",io.from_pfQ.fire)
  XSPerfAccumulate("hyper_filter_output",io.out_mlfq2.fire || io.out_mlfq1.fire)
  XSPerfAccumulate("hyper_filter_mlfq1",io.out_mlfq1.fire)
  XSPerfAccumulate("hyper_filter_mlfq1_bop",io.out_mlfq1.fire && io.out_mlfq1.bits.pfVec === FitlerVecState.BOP)
  XSPerfAccumulate("hyper_filter_mlfq1_spp",io.out_mlfq1.fire && io.out_mlfq1.bits.pfVec === FitlerVecState.SPP)
  XSPerfAccumulate("hyper_filter_mlfq2",io.out_mlfq2.fire)
  XSPerfAccumulate("hyper_filter_evict_fomMshr",io.evict.fire)
  XSPerfAccumulate("hyper_filter_evict_fromQ",hitEvict)

}

//Only used for hybrid spp and bop and sms
class HyperPrefetcher(parentName:String = "Unknown")(implicit p: Parameters) extends PrefetchBranchV2Module with HasPerfLogging{
  val io = IO(new Bundle() {
    val train = Flipped(DecoupledIO(new PrefetchTrain))
    val req = DecoupledIO(new PrefetchReq)
    val resp = Flipped(DecoupledIO(new PrefetchResp))
    val evict = Flipped(DecoupledIO(new PrefetchEvict))
    val recv_addr = Flipped(ValidIO(UInt(64.W)))
    val hint2llc = ValidIO(new PrefetchReq)
    // val db_degree = Flipped(ValidIO(UInt(2.W)))
    // val queue_used = Input(UInt(6.W))
  })
  dontTouch(io.resp.bits)
  // --------------------------------------------------------------------------------
  // global counter 
  // --------------------------------------------------------------------------------
  val (counterValue, counterWrap) = Counter(true.B, 1024)
  val deadPfEviction = RegInit(0.U(13.W))
  val issued = RegInit(0.U(16.W))
  val pf_state = WireInit(0.U(2.W));dontTouch(pf_state)
  when(io.evict.valid && io.evict.bits.is_prefetch) {
    deadPfEviction := deadPfEviction + 1.U
  }
  when(io.req.fire){
    issued := issued + 1.U
  }
  when(counterWrap) {
    deadPfEviction := 0.U
    issued := 0.U
    // deadPfEviction/issued > 0.75, 
    when((deadPfEviction << 2) > issued + issued + issued) {
      pf_state := 3.U
    } .elsewhen((deadPfEviction << 1) > issued) {
      pf_state := 2.U
    } .elsewhen((deadPfEviction << 2) > issued) {
      pf_state := 1.U
    } .otherwise {
      pf_state := 0.U
    }
  }
  XSPerfHistogram("prefetch_dead_block", deadPfEviction, counterWrap, 0, 200, 5)
  XSPerfHistogram("prefetch_dead_ratio", pf_state, counterWrap, 0, 4, 1)
  // --------------------------------------------------------------------------------
  // multi prefetch buffer Queue
  // --------------------------------------------------------------------------------
  val q_bop = Module(new Queue(new PrefetchReq, 4, flow = true,  pipe = false))
  val q_spp = Module(new Queue(new PrefetchReq, 4, flow = true, pipe = false))
  val q_sms = Module(new Queue(new PrefetchReq, 4, flow = true, pipe = false))

  // --------------------------------------------------------------------------------
  // multi-level feedback Queue
  // --------------------------------------------------------------------------------
  // | MLFQ_1 | ->
  // | MLFQ_2 | -->
  // | MLFQ_3 | ---> firstly
  // val MLFQ_1 = Module(new Queue(new PrefetchReq, 32, flow = false, pipe = false))
  // val MLFQ_2 = Module(new Queue(new PrefetchReq, 16, flow = false, pipe = false))
  val MLFQ_1 = Module(new Monitorbuffer(new PrefetchReq, 32))
  val MLFQ_2 = Module(new ReplaceableQueueV2(new PrefetchReq, 16))
  val MLFQ_3 = Module(new Queue(new PrefetchReq, 4,  flow = true,  pipe = false))


  // dontTouch(MLFQ_1.io.used)
  // dontTouch(MLFQ_2.io.used)

  MLFQ_3.io.enq.valid := false.B
  MLFQ_3.io.enq.bits := 0.U.asTypeOf(new PrefetchReq)

  io.req.valid := MLFQ_3.io.deq.fire || MLFQ_2.io.deq.fire || MLFQ_1.io.deq.fire
  io.req.bits := ParallelPriorityMux(
    Seq(
      MLFQ_3.io.deq.valid -> MLFQ_3.io.deq.bits,
      MLFQ_2.io.deq.valid -> MLFQ_2.io.deq.bits,
      MLFQ_1.io.deq.valid -> MLFQ_1.io.deq.bits
    )
  )
  MLFQ_3.io.deq.ready := true.B
  MLFQ_2.io.deq.ready := !MLFQ_3.io.deq.fire
  MLFQ_1.io.deq.ready := !MLFQ_3.io.deq.fire && !MLFQ_2.io.deq.fire
  // --------------------------------------------------------------------------------
  // stage 0 
  // --------------------------------------------------------------------------------
  // | q_sms | q_bop | q_spp |
  val fTable = Module(new FilterTable(parentName + "ftable_"))

  val spp = Module(new prefetch.SignaturePathPrefetch(parentName = parentName + "spp_")(p.alterPartial({
        case L2ParamKey => p(L2ParamKey).copy(prefetch = Some(SPPParameters()))
  })))
  val bop = Module(new BestOffsetPrefetch()(p.alterPartial({
        case L2ParamKey => p(L2ParamKey).copy(prefetch = Some(BOPParameters()))
  })))
  val sms = Module(new PrefetchReceiver()(p.alterPartial({
        case L2ParamKey => p(L2ParamKey).copy(prefetch = Some(PrefetchReceiverParams()))
  })))

  q_bop.io.enq <> bop.io.req
  q_sms.io.enq <> sms.io.req
  q_spp.io.enq <> spp.io.req

  val fTable_req = WireInit(0.U.asTypeOf(new PrefetchReq))
  fTable_req := ParallelPriorityMux(
    Seq(
      q_spp.io.deq.valid  -> q_spp.io.deq.bits,
      q_bop.io.deq.valid  -> q_bop.io.deq.bits,
      q_sms.io.deq.valid  -> q_sms.io.deq.bits
    )
  )
  // fastArb(Seq(q_bop.io.deq,q_spp.io.deq,q_sms.io.deq),fTable.io.from_pfQ)
  // val (counterValue_MLFQ, counterWrap_MLFQ) = Counter(!q_bop.io.count === 0.U, 8)
  q_spp.io.deq.ready := true.B
  q_bop.io.deq.ready := !q_spp.io.deq.fire
  q_sms.io.deq.ready := !q_bop.io.deq.fire && !q_spp.io.deq.fire

  fTable.io.from_mlfq1.valid := false.B
  fTable.io.from_mlfq1.bits := 0.U.asTypeOf(fTable.io.from_mlfq1.bits.cloneType)

  fTable.io.from_pfQ.valid := q_bop.io.deq.fire || q_spp.io.deq.fire || q_sms.io.deq.fire
  fTable.io.from_pfQ.bits := fTable_req
  fTable.io.from_pfQ.ready := DontCare


  fTable.io.from_mlfq1 <> MLFQ_1.io.toFilter
  MLFQ_1.io.fromFilter <> fTable.io.qurryResp_mlfq1

  //directly flow 
  fTable.io.from_sppHintQ <> spp.io.hint_req


  MLFQ_1.io.enq <> fTable.io.out_mlfq1
  MLFQ_2.io.enq <> fTable.io.out_mlfq2
  

  // --------------------------------------------------------------------------------
  // train diverter queue
  // --------------------------------------------------------------------------------
  val train_q = Module(new Queue(new PrefetchTrain, entries = 4, flow = true, pipe = false))
  val train_bop_q = Module(new Queue(new PrefetchTrain, entries = 2, flow = true, pipe = false))
  val train_spp_q = Module(new Queue(new PrefetchTrain, entries = 2, flow = true, pipe = false))
  dontTouch(train_q.io.count)
  dontTouch(train_bop_q.io.count)
  dontTouch(train_spp_q.io.count)
  train_q.io.enq <> io.train

  train_bop_q.io.enq.valid := train_q.io.deq.valid && train_q.io.deq.bits.hasBOP && (train_q.io.deq.bits.state === AccessState.MISS || train_q.io.deq.bits.state === AccessState.PREFETCH_HIT)
  train_bop_q.io.enq.bits := train_q.io.deq.bits

  train_spp_q.io.enq.valid := train_q.io.deq.valid && train_q.io.deq.bits.hasSPP && (train_q.io.deq.bits.state === AccessState.MISS || train_q.io.deq.bits.state === AccessState.PREFETCH_HIT)
  train_spp_q.io.enq.bits := train_q.io.deq.bits

  train_q.io.deq.ready := true.B

  bop.io.train.valid := train_bop_q.io.deq.valid
  bop.io.train.bits := train_bop_q.io.deq.bits
  train_bop_q.io.deq.ready := bop.io.train.ready

  bop.io.resp.valid := io.resp.valid && io.resp.bits.hasBOP
  bop.io.resp.bits := io.resp.bits
  io.resp.ready := bop.io.resp.ready

  spp.io.train.valid := train_spp_q.io.deq.valid
  spp.io.train.bits := train_spp_q.io.deq.bits
  train_spp_q.io.deq.ready := spp.io.train.ready

  spp.io.resp.valid := false.B
  spp.io.resp.bits.tag := 0.U
  spp.io.resp.bits.set := 0.U
  spp.io.resp.bits.pfVec := DontCare
  
  // --------------------------------------------------------------------------------
  // stage 1
  // --------------------------------------------------------------------------------

  spp.io.req.ready := true.B
  bop.io.req.ready := true.B

  sms.io.recv_addr.valid := io.recv_addr.valid
  sms.io.recv_addr.bits := io.recv_addr.bits
  sms.io.req.ready := true.B

  // io.hint2llc := fTable.io.hint2llc;dontTouch(io.hint2llc)
  io.hint2llc := 0.U.asTypeOf(Valid(new PrefetchReq))
  fTable.io.evict.valid := io.evict.valid
  fTable.io.evict.bits := io.evict.bits
  io.evict.ready := fTable.io.evict.ready

  // fTable.io.from_bop := bop.io.req.valid
  dontTouch(io.train.bits)
  io.train.ready := true.B
  spp.io.db_degree.valid := false.B
  spp.io.db_degree.bits := counterWrap
  spp.io.queue_used := pf_state

  XSPerfAccumulate("bop_recv_train", train_bop_q.io.deq.fire)
  XSPerfAccumulate("spp_recv_train", train_spp_q.io.deq.fire)
  XSPerfAccumulate("bop_send2_queue", q_bop.io.enq.fire)
  XSPerfAccumulate("sms_send2_queue", q_sms.io.enq.fire)
  XSPerfAccumulate("spp_send2_queue", q_spp.io.enq.fire)
  XSPerfAccumulate("sms_q_deq", q_sms.io.deq.fire)
  XSPerfAccumulate("spp_q_deq", q_spp.io.deq.fire)
  // XSPerfAccumulate("hyper_overlapped", (bop.io.req.valid || q_spp.io.deq.valid || q_sms.io.deq.valid) && fTable.io.req.ready)
  XSPerfAccumulate("prefetcher_has_evict", io.evict.fire)
}