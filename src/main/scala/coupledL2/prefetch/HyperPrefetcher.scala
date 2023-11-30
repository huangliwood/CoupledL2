package coupledL2.prefetch

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import freechips.rocketchip.tilelink._
import coupledL2._
import coupledL2.HasCoupledL2Parameters
import xs.utils.perf.HasPerfLogging
import xs.utils.SRAMQueue

case class HyperPrefetchParams(
  fTableEntries: Int = 32,
  pTableQueueEntries: Int = 2,
  fTableQueueEntries: Int = 256
)
    extends PrefetchParameters {
  override val hasPrefetchBit:  Boolean = true
  override val inflightEntries: Int = 64
}

trait HasHyperPrefetcherParams extends HasCoupledL2Parameters {
  val hyperPrefetchParams = prefetchOpt.get.asInstanceOf[HyperPrefetchParams]

  val pageAddrBits = fullAddressBits - pageOffsetBits
  val blkOffsetBits = pageOffsetBits - offsetBits

  val fTableEntries = hyperPrefetchParams.fTableEntries
  val fTagBits = pageAddrBits - log2Up(fTableEntries)
  val pTableQueueEntries = hyperPrefetchParams.pTableQueueEntries
  val fTableQueueEntries = hyperPrefetchParams.fTableQueueEntries
}

abstract class PrefetchBranchV2Module(implicit val p: Parameters) extends Module with HasHyperPrefetcherParams
abstract class PrefetchBranchV2Bundle(implicit val p: Parameters) extends Bundle with HasHyperPrefetcherParams

object PrefetcherId {
  val bits = 2

  def BOP           = 0.U(bits.W)
  def SMS           = 1.U(bits.W)
  def SPP           = 2.U(bits.W)
  def OHTERS        = 3.U(bits.W)
}

class FilterTiming(parentName:String="Unkown")(implicit p: Parameters) extends PrefetchBranchV2Module {
 val io = IO(new Bundle() {
   val req = Flipped(DecoupledIO(new PrefetchReq))
   val resp = DecoupledIO(new PrefetchReq)
   val evict = Flipped(DecoupledIO(new PrefetchEvict))
   val from_bop = Input(Bool())
   val spp2llc = Input(Bool())
   val hint2llc = ValidIO(new PrefetchReq)
 })

 def idx(addr:      UInt) = addr(log2Up(fTableEntries) - 1, 0)
 def tag(addr:      UInt) = addr(pageAddrBits - 1, log2Up(fTableEntries))

 def fTableEntry() = new Bundle {
   val valid = Bool()
   val tag = UInt(fTagBits.W)
   val bitMap = Vec(64, Bool())
 }
 val dupNums = 16

 val req_dups = RegInit(VecInit(Seq.fill(dupNums)(0.U.asTypeOf(new PrefetchReq))))
 val req_dups_valid = RegInit(VecInit(Seq.fill(dupNums)(false.B)))
 val req_hint2llc = RegNext(io.spp2llc,false.B)
 req_dups.foreach(_ := io.req.bits)
 req_dups_valid.foreach( _ := io.req.valid)
 val dupOffsetBits = log2Up(fTableEntries/dupNums)
 val dupBits = log2Up(dupNums)
 val fTable = RegInit(VecInit(Seq.fill(fTableEntries)(0.U.asTypeOf(fTableEntry()))))
 val q = Module(new Queue(UInt(fullAddressBits.W), fTableQueueEntries,flow= false, pipe= false))
  //  val q = Module(new SRAMQueue(UInt(fullAddressBits.W), fTableQueueEntries,
  //    flow = true, hasMbist = cacheParams.hasMbist, hasShareBus = cacheParams.hasShareBus,
  //    hasClkGate = enableClockGate, parentName = parentName))

 val hit = WireInit(VecInit.fill(dupNums)(false.B))
 val readResult = WireInit(VecInit.fill(dupNums)(0.U.asTypeOf(fTableEntry())))
 val hitForMap = WireInit(VecInit.fill(dupNums)(false.B))

 for(i <- 0 until(dupNums)) {
   when(req_dups(i).set(dupOffsetBits-1+dupBits-1,dupOffsetBits-1) === i.U(dupBits.W)) {
     val oldAddr = req_dups(i).addr
     val pageAddr = oldAddr(fullAddressBits - 1, pageOffsetBits)
     val blkOffset = oldAddr(pageOffsetBits - 1, offsetBits)

     //read fTable
     readResult(i) := fTable(idx(pageAddr))
     hit(i) := readResult(i).valid
     hitForMap(i) := hit(i) && readResult(i).bitMap(blkOffset)

     val wData = WireInit(0.U.asTypeOf(fTableEntry()))
     val newBitMap = readResult(i).bitMap.zipWithIndex.map { case (b, i) => Mux(i.asUInt === blkOffset, true.B, false.B) }

     wData.valid := true.B
     wData.tag := tag(pageAddr)
     wData.bitMap := newBitMap
     when(req_dups_valid(i)) {
       when(hit(i)) {
         fTable(idx(pageAddr)).bitMap(blkOffset) := true.B
       }.otherwise {
         fTable(idx(pageAddr)) := wData
       }
     }
   }
 }
 io.resp.valid := req_dups_valid(0) && (!hitForMap.asUInt.orR || io.from_bop)
 io.resp.bits := req_dups(0)

 io.hint2llc.valid := req_dups_valid(1) && req_hint2llc
 io.hint2llc.bits := req_dups(1)

 q.io.enq.valid := req_dups_valid(2) && !hitForMap.asUInt.orR && !req_hint2llc // if spp2llc , don't enq
 q.io.enq.bits := req_dups(2).addr

 val isqFull = q.io.count === fTableQueueEntries.U
 q.io.deq.ready := isqFull;dontTouch(q.io.deq.ready)

 val evictAddr = q.io.deq.bits
 val evictPageAddr = evictAddr(fullAddressBits - 1, pageOffsetBits)
 val evictBlkOffset = evictAddr(pageOffsetBits - 1, offsetBits)
 val evictBlkAddr = evictAddr(fullAddressBits - 1, offsetBits)
 val readEvict = WireInit(VecInit.fill(dupNums)(0.U.asTypeOf(fTableEntry())))
 val hitEvict =  WireInit(VecInit.fill(dupNums)(false.B))
 for(i <- 0 until(dupNums)) {
   when(req_dups(i).set(dupOffsetBits-1+dupBits,dupOffsetBits-1) === i.U(dupBits.W)) {
     val oldAddr = req_dups(i).addr
     val blkAddr = oldAddr(fullAddressBits - 1, offsetBits)
     val conflict = req_dups_valid.reduce(_ || _) && blkAddr === evictBlkAddr
     readEvict(i) := fTable(idx(evictPageAddr))
     hitEvict(i) := RegEnable(readEvict(i).valid && tag(evictPageAddr) === readEvict(i).tag && readEvict(i).bitMap(evictBlkOffset) && !conflict, q.io.deq.fire)
     when(hitEvict(i)) {
       fTable(idx(evictPageAddr)).bitMap(evictBlkOffset) := false.B
     }
   }
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
 io.req.ready := true.B
 io.evict.ready := true.B
 XSPerfAccumulate("hyper_filter_valid_nums", req_dups_valid(0) && (hitForMap.asUInt.orR))
 XSPerfAccumulate("hyper_filter_input", io.req.fire)
 XSPerfAccumulate("hyper_filter_output", io.resp.fire)
}


//Only used for hybrid spp and bop
class HyperPrefetcher(parentName:String = "Unknown")(implicit p: Parameters) extends PrefetchBranchV2Module with HasPerfLogging{
  val io = IO(new Bundle() {
    val train = Flipped(DecoupledIO(new PrefetchTrain))
    val req = DecoupledIO(new PrefetchReq)
    val resp = Flipped(DecoupledIO(new PrefetchResp))
    val evict = Flipped(DecoupledIO(new PrefetchEvict))
    val recv_addr = Flipped(ValidIO(UInt(64.W)))
    val hint2llc = ValidIO(new PrefetchReq)
    val db_degree = Flipped(ValidIO(UInt(2.W)))
    val queue_used = Input(UInt(6.W))
  })

  val fTable = Module(new FilterTiming(parentName + "ftable_"))

  val spp = Module(new SignaturePathPrefetch(parentName = parentName + "spp_")(p.alterPartial({
        case L2ParamKey => p(L2ParamKey).copy(prefetch = Some(SPPParameters()))
  })))
  val bop = Module(new BestOffsetPrefetch()(p.alterPartial({
        case L2ParamKey => p(L2ParamKey).copy(prefetch = Some(BOPParameters()))
  })))
  val sms = Module(new PrefetchReceiver()(p.alterPartial({
        case L2ParamKey => p(L2ParamKey).copy(prefetch = Some(PrefetchReceiverParams()))
  })))

  val q_sms = Module(new Queue(chiselTypeOf(sms.io.req.bits), pTableQueueEntries, flow = true, pipe = false))
  val q_spp = Module(new Queue(chiselTypeOf(spp.io.req.bits), pTableQueueEntries, flow = false, pipe = false))
  q_sms.io.enq <> sms.io.req
  q_sms.io.deq.ready := !bop.io.req.valid

  q_spp.io.enq <> spp.io.req
  q_spp.io.deq.ready := !q_sms.io.deq.fire && !bop.io.req.valid

  spp.io.train.valid := io.train.valid && io.train.bits.state === AccessState.MISS
  spp.io.train.bits := io.train.bits

  val train_for_bop = RegInit(0.U.asTypeOf(new PrefetchTrain))
  val train_for_bop_valid = RegInit(false.B)
  
  when(io.train.valid && !bop.io.train.ready) {
    train_for_bop := io.train.bits
    train_for_bop_valid := true.B
  }
  bop.io.train.valid := io.train.valid || train_for_bop_valid
  bop.io.train.bits := Mux(io.train.valid, io.train.bits, train_for_bop)
  when(bop.io.train.fire && !io.train.valid) {
    train_for_bop_valid := false.B
  }

  bop.io.resp <> io.resp
  spp.io.resp.bits.tag := 0.U
  spp.io.resp.bits.set := 0.U
  spp.io.resp.valid := false.B

  spp.io.req.ready := true.B
  bop.io.req.ready := true.B

  sms.io.recv_addr.valid := io.recv_addr.valid
  sms.io.recv_addr.bits := io.recv_addr.bits
  sms.io.req.ready := true.B

  when(bop.io.req.valid) {
    fTable.io.req <> bop.io.req
    fTable.io.pf_Id := PrefetcherId.BOP
    fTable.io.isForce := true.B
    // fTable.io.spp2llc := false.B
  }.elsewhen(q_sms.io.deq.valid) {
    fTable.io.req <> q_sms.io.deq
    fTable.io.pf_Id := PrefetcherId.SMS
    fTable.io.isForce := true.B
    // fTable.io.spp2llc := false.B
  }.otherwise {
    fTable.io.req.valid := RegNext(q_spp.io.deq.valid,false.B)
    fTable.io.req.bits.tag := RegEnable(q_spp.io.deq.bits.tag,q_spp.io.deq.valid)
    fTable.io.req.bits.set := RegEnable(q_spp.io.deq.bits.set,q_spp.io.deq.valid)
    fTable.io.req.bits.isBOP := RegEnable(q_spp.io.deq.bits.isBOP,q_spp.io.deq.valid)
    fTable.io.req.bits.source := RegEnable(q_spp.io.deq.bits.source,q_spp.io.deq.valid)
    fTable.io.req.bits.needT := RegEnable(q_spp.io.deq.bits.needT,q_spp.io.deq.valid)
    fTable.io.pf_Id := PrefetcherId.SPP
    fTable.io.isForce := false.B
    // fTable.io.spp2llc := q_spp.io.deq.bits.hint2llc
  }

  io.req <> fTable.io.resp
  // io.hint2llc := fTable.io.hint2llc;dontTouch(io.hint2llc)
  io.hint2llc := 0.U.asTypeOf(Valid(new PrefetchReq))
  fTable.io.evict.valid := io.evict.valid
  fTable.io.evict.bits := io.evict.bits
  io.evict.ready := fTable.io.evict.ready

  // fTable.io.from_bop := bop.io.req.valid

  io.train.ready := true.B
  spp.io.db_degree.valid := io.db_degree.valid
  spp.io.db_degree.bits := io.db_degree.bits
  spp.io.queue_used := io.queue_used

  XSPerfAccumulate("bop_send2_queue", bop.io.req.valid)
  XSPerfAccumulate("sms_send2_queue", q_sms.io.enq.fire)
  XSPerfAccumulate("spp_send2_queue", q_spp.io.enq.fire)
  XSPerfAccumulate("sms_q_deq", q_sms.io.deq.fire)
  XSPerfAccumulate("spp_q_deq", q_spp.io.deq.fire)
  XSPerfAccumulate("hyper_overlapped", (bop.io.req.valid || q_spp.io.deq.valid || q_sms.io.deq.valid) && fTable.io.req.ready)
  XSPerfAccumulate("prefetcher_has_evict", io.evict.fire)
}