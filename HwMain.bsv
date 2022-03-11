import FIFO::*;
import FIFOF::*;
import Clocks::*;
import Vector::*;
import BRAM::*;
import BRAMFIFO::*;

import PcieCtrl::*;
import DRAMController::*;
import Serializer::*;
import FIFOLI::*;

import BLRadix::*;
interface HwMainIfc;
endinterface

module mkHwMain#(PcieUserIfc pcie, DRAMUserIfc dram) 
    (HwMainIfc);
	BLRadixIfc#(8,3,4,Bit#(32),0,7) radixSub <- mkBLRadix;

    Reg#(Bit#(32)) file_size <- mkReg(0);
    Reg#(Bit#(32)) dramWriteCnt <- mkReg(0);
    Reg#(Bit#(32)) dramReadCnt <- mkReg(0);
    FIFOLI#(Tuple2#(Bit#(20), Bit#(32)), 2) pcie_reqQ <- mkFIFOLI;

    SerializerIfc#(128, 4) serial_pcieio <- mkSerializer;
    SerializerIfc#(128, 16) serial_input <- mkSerializer;
    DeSerializerIfc#(128, 4) deserial_dram <- mkDeSerializer;
    SerializerIfc#(512, 4) serial_dramQ <- mkSerializer;

    FIFO#(Bit#(32)) dmaReadReqQ <- mkFIFO;
    FIFO#(Bit#(32)) dmaWriteReqQ <- mkFIFO;
    FIFO#(Bit#(1)) dmaWriteDoneSignalQ <- mkFIFO;
    FIFO#(Bit#(128)) dma_inQ <- mkFIFO;

    Reg#(Bit#(32)) readCnt <- mkReg(0);

    Reg#(Bit#(1)) dmaWriteHandle <- mkReg(0);
    Reg#(Bit#(32)) dmaWriteTarget <- mkReg(0);
    Reg#(Bit#(32)) dmaWriteCnt <- mkReg(0);

    rule getDataFromHost;
        let w <- pcie.dataReceive;
        let a = w.addr;
        let d = w.data;
        pcie_reqQ.enq(tuple2(a, d));
    endrule

    rule getPCIeData; // get from HOST
        pcie_reqQ.deq;
        Bit#(20) a = tpl_1(pcie_reqQ.first);
        Bit#(32) d = tpl_2(pcie_reqQ.first);

        let off = (a>>2);
        if ( off == 0 ) begin
            file_size <= d;
        end else if (off == 1) begin // Log Data In
            dmaReadReqQ.enq(d);
        end else begin
            $display("Wrong PCIe Signal");
        end
    endrule

    rule getReadReq(readCnt == 0);
        dmaReadReqQ.deq;
        Bit#(32) cnt = dmaReadReqQ.first;
        pcie.dmaReadReq(0, truncate(cnt)); // offset, words
        readCnt <= cnt;
    endrule


    rule getDataFromDMA(readCnt != 0);
        Bit#(128) rd <- pcie.dmaReadWord;
        if (readCnt - 1 == 0) begin
            dmaWriteDoneSignalQ.enq(1);
        end
        readCnt <= readCnt - 1;
        dma_inQ.enq(rd);
    endrule

	Reg#(Bit#(32)) dataInputCounter <- mkReg(0);
	FIFO#(Vector#(4, Bit#(32))) toUploader <- mkFIFO;

	Integer dataCnt = 1024*512;

	rule inputData;
	    dma_inQ.deq;
        Bit#(128) t = dma_inQ.first;
		dataInputCounter <= dataInputCounter + 1;
		Vector#(4,Bit#(32)) ind;
		ind[0] = t[127:96];
		ind[1] = t[95:64];
		ind[2] = t[63:32];
		ind[3] = t[31:0];
		radixSub.enq(ind);
	endrule

	Reg#(Bit#(32)) burstTotal <- mkReg(0);
	Reg#(Bit#(32)) startCycle <- mkReg(0);
	rule flushBurstReady;
		let d <- radixSub.burstReady;
		burstTotal <= burstTotal + zeroExtend(d);
	endrule

	Reg#(Bit#(32)) dataOutputCounter <- mkReg(0);
	rule readOutput;
		Vector#(4,Bit#(32)) outd = radixSub.first;
		radixSub.deq;
		toUploader.enq(outd);
		dataOutputCounter <= dataOutputCounter + 1;
	endrule


    Vector#(4, FIFO#(Bit#(32))) toNodeQ <- replicateM(mkFIFO);
    Reg#(Bit#(32)) run_cnt <- mkReg(0); 
    Reg#(Bit#(32)) clock_cnt <- mkReg(0); 

    rule get_Data_from_radix_sorter;
        toUploader.deq;
        Vector#(4, Bit#(32)) t = toUploader.first;
        Bit#(128) d = 0;
        
		d[127:96] = t[0];
		d[95:64] = t[1]; 
		d[63:32] = t[2]; 
		d[31:0] = t[3]; 

		deserial_dram.put(d);
    endrule

    /* Write to DRAM */
    rule dramWrite(dramWriteCnt < file_size);
        dramWriteCnt <= dramWriteCnt + 64;
        Bit#(512) d <- deserial_dram.get;
        dram.write(zeroExtend(dramWriteCnt), d, 64);
    endrule

    /* DRAM read */
    rule dramReadReq(dramWriteCnt >= file_size - 64 && dramReadCnt < file_size);
        dramReadCnt <= dramReadCnt + 64;
        dram.readReq(zeroExtend(dramReadCnt), 64);
    endrule
    rule dramRead;
        Bit#(512) d <- dram.read;
    endrule

    /* Giving DMA write done signal to the HOST */
    rule sendResultToHost; 
        let r <- pcie.dataReq;
        let a = r.addr;
        let offset = (a>>2);
        if ( offset == 0 ) begin
            pcie.dataSend(r, 1);
        end else begin
            dmaWriteDoneSignalQ.deq;
            pcie.dataSend(r, 1);
        end
    endrule
endmodule
