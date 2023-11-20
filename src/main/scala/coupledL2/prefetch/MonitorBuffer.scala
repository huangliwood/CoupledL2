package coupledL2.prefetch
import chisel3._
import chisel3.util._
import xs.utils.{CircularShift,CircularQueuePtr,HasCircularQueuePtrHelper}
import org.chipsalliance.cde.config.Parameters
import coupledL2.HasCoupledL2Parameters
import coupledL2.PfSource

class Monitorbuffer[T <: Data](val gen: PrefetchReq,val entries:Int=16)(implicit val p: Parameters)
extends Module with HasCircularQueuePtrHelper with HasHyperPrefetcherParams{
    val io = IO(new Bundle{
        val enq = Flipped(DecoupledIO(gen))
        val deq = DecoupledIO(gen)
        val used = Output(UInt(log2Up(entries).W))
        val toFilter = DecoupledIO(gen)
        val fromFilter = Flipped(ValidIO(new Bundle {
            val needDrop = Bool()
        }))
    })
    val enq_num = 1
    val deq_num = 1
    class MonitorbufferPtr(implicit p: Parameters) extends CircularQueuePtr[MonitorbufferPtr](entries){}
    val enqPtr = RegInit(0.U.asTypeOf(new MonitorbufferPtr))
    val deqPtr = RegInit(0.U.asTypeOf(new MonitorbufferPtr))

    val skip_deq = WireInit(false.B)
    
    def entry_map[T](fn: Int => T) = (0 until entries).map(fn)
    val entries_valid = RegInit(VecInit(Seq.fill(entries)(false.B)))
    val entries_data = RegInit(VecInit(Seq.fill(entries)(0.U.asTypeOf(gen))))


    val empty = isEmpty(enq_ptr = enqPtr, deq_ptr = deqPtr)
    val full  = isFull(enq_ptr = enqPtr, deq_ptr = deqPtr)

    when(io.enq.valid) {
        enqPtr := enqPtr + 1.U
        when(full && !io.deq.ready) {
            deqPtr := deqPtr + 1.U
        }

        for(i <- 0 until entries){
            when(i.U === enqPtr.value){
                entries_valid(i) := true.B
                entries_data(i) := io.enq.bits
            }
        }
    }
    when(!empty && (io.deq.fire||skip_deq)){
        for(i <- 0 until entries){
            when(i.U === deqPtr.value){
                entries_valid(i) := false.B
            }
        }
        deqPtr := deqPtr + 1.U
    }
    // --------------------------------------------------------------------------------
    // dequeue operation
    // --------------------------------------------------------------------------------
    // 1. firstly qurry filterTable is existing overlapped prefetchReq
    // 2. if existed, directly throw out and dequeue

    val s_idle :: s_qurryFilter :: Nil = Enum(2)
    val state = RegInit(s_idle)
    val (counterValue, counterWrap) = Counter(!empty, 8)
    val direct_deq = WireInit(entries_data(deqPtr.value).pfVec === PfSource.SMS || entries_data(deqPtr.value).pfVec === PfSource.SPP)
    switch(state){
        is(s_idle){
            when(counterWrap && !direct_deq){
                state := s_qurryFilter
            }
        }
        is(s_qurryFilter){
            when(io.fromFilter.valid){
                state := s_idle
            }
        }
    }
    val can_deq = RegInit(false.B)
    can_deq := io.fromFilter.valid && io.fromFilter.bits.needDrop && !io.deq.ready

    io.toFilter.valid := state === s_qurryFilter && !io.fromFilter.valid
    io.toFilter.bits := entries_data(deqPtr.value)

    io.enq.ready := true.B
    skip_deq := io.fromFilter.valid && io.fromFilter.bits.needDrop
    io.deq.valid := (io.fromFilter.valid && !io.fromFilter.bits.needDrop && io.deq.ready) || can_deq || direct_deq
    io.deq.bits := entries_data(deqPtr.value)
    io.used := distanceBetween(enqPtr, deqPtr)

}