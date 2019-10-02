
module counter8( d, clk, clear, load, count, qd);


input   [7:0] d;
input   clk;
input   clear;
input   load;
input   count;
output  [7:0] qd;

reg     [7:0] cnt;


always @ (posedge clk or negedge clear)

    if (!clear)
        cnt = 8'b00000000;
    else if (load)
        cnt = d;
    else if (count)
        cnt = cnt + 1;
    
 

assign  qd = cnt;

endmodule



module Moore_blink (AT,BT,finished,Traffic,AB[1],AB[0],CLK,RST);
input finished,AT,BT,CLK,RST,Traffic;
//output reg e;
output  [1:0]AB;
reg [1:0] state;
parameter S0 = 2'b10, S1 = 2'b00, S2 = 2'b11;
always @ (posedge CLK or negedge RST)
if (~RST) state = S0; 
else
case (state)
S0: if (~AT&~BT&finished) state = S1;
S1: if (~AT&~BT|~Traffic) state = S2; else if (Traffic) state=S0;
S2: if (~AT&~BT|~Traffic) state = S1; else if(Traffic) state=S0;
endcase

assign #1 AB = state;        
endmodule



module blink (finishedb,AL,BL,e,PO,AT,BT,CLK,RST);
  input AT,BT,PO,CLK,RST;
  output AL,BL;
  output  e,finishedb;
  
  wire load;
  wire [7:0]Q;
  wire [7:0]K;
  wire Traffic;
  Moore_blink mb(AT,BT,finishedb,Traffic,AL,BL,CLK,RST);
  counter8 cnt125(8'b00000000, CLK, RST, load|PO, count, Q);
  counter8 cnt(8'b00000000, CLK, RST, (Traffic|(~AT&~BT))|PO, AT|BT, K);
  assign Traffic=K[0]&K[2];
  assign finishedb=Q[0]&~Q[1]&Q[2]&Q[3]&Q[4]&Q[5]&Q[6]&~Q[7];
  
  assign load=~(AL^BL);
  assign count=AL&~BL;
  assign e=~(AL^BL);
endmodule




module test_b;
  reg AT,BT,CLK,RST,PO;
  wire AL,BL,e,finished;
  
 // wire [7:0]Q;
  
 // reg [7:0]d,clk,clear,load,up_down;
  //wire [7:0]qd;
  
  blink b(finished,AL,BL,e,PO,AT,BT,CLK,RST);

  initial
     begin
         RST = 0;
         CLK = 0;
      #15 RST = 1; 
         repeat (1000)
      #2 CLK = ~CLK;
     end
initial 
begin
  PO=0;
  AT=0;
  BT=1;
  #100
  BT=0;
  #700
  AT=1;
  #100 AT=0;
  #700 AT=1;
  #200 AT=0;
  
end
endmodule


//
//Reverse counter 
//

module reverse( d, clk,reset, clear, load, count, qd);


input   [3:0] d;
input   clk;
input   clear;
input   load;
input   count,reset;
output  [3:0] qd;

reg     [3:0] cnt;


always @ (posedge clk or negedge reset)

    if (!reset)
        cnt = d;
    else if (load)
        cnt = d;
       else if (clear)
         cnt=4'b1001;
    else if (count)
        cnt = cnt - 1;
    
 

assign qd = cnt;

endmodule

module two_bit_bcd(Q1,Q0,load,count,D1,D0,CLK,RST);
  output [3:0]Q1;
  output [3:0]Q0;
  input load,count,CLK,RST;
  input [3:0]D1;
  input [3:0]D0;
  wire cnt0,cnt1;
  assign cnt1= ~(Q1[0]|Q1[1]|Q1[2]|Q1[3]);
  assign cnt0= ~(Q0[0]|Q0[1]|Q0[2]|Q0[3]);
  reverse rv1(D0,CLK,RST,cnt0,load,count,Q0);
  reverse rv2(D1,CLK,RST,cnt1&cnt0,load,cnt0,Q1);
  
endmodule  

module test_counter;
  reg load,count,CLK,RST;
  reg [3:0]D0;
  reg [3:0]D1;
  wire [3:0]Q0;
  wire [3:0]Q1;
  
  two_bit_bcd tb(Q1,Q0,load,count,D1,D0,CLK,RST);
  
  initial 
  begin
    RST=0;
    CLK=0;
    #10
    RST=1;
    repeat(60)
  #5  CLK=~CLK;
    
  end
  
  
  initial 
  begin
    load=1;
    count=1;
    D0=4'b1001;
    D1=4'b0010;
    #10 load=0;
  end
endmodule

module mux4x1 (i0,i1,i2,i3,select[1],select[0],y);
   input [3:0]i0;
   input [3:0]i1;
   input [3:0]i2;
   input [3:0]i3;
   input [1:0] select;
   output reg [3:0]y;
   
   always @ (i0 or i1 or i2 or i3 or select) 
            case (select)
               2'b00: y = i0;
               2'b01: y = i1;
               2'b10: y = i2;
               2'b11: y = i3;
            endcase
endmodule


module number_set (D1,D0,EN,L0,L1,L2,L3,CLK,RST);
  input EN,L0,L1,L2,L3,CLK,RST;
  output [3:0]D0;
  output [3:0]D1;
  wire [3:0]ATL0;
  wire [3:0]ATH0;
  wire [3:0]ATL1;
  wire [3:0]ATH1;
  wire [3:0]ATL2;
  wire [3:0]ATH2;
  
 // mux4x1 mx0(4'b1010,4'b0101,4'b1110,4'b0101,(L3|L2),(L3|L1),D0);
//  mux4x1_bh mx1(4'b0101,4'b0000,4'b0001,4'b0000,(L3|L2),(L3|L1),D1);
  
  two_bit_bcd cnt0(ATH0,ATL0,~L0,L0&EN,4'b1001,4'b0000,CLK,RST);
  two_bit_bcd cnt1(ATH1,ATL1,~(L1|L3),(L1|L3)&EN,4'b0000,4'b0101,CLK,RST);
  two_bit_bcd cnt2(ATH2,ATL2,~L2,L2&EN,4'b0011,4'b0000,CLK,RST);
  
  mux4x1 mx0(ATH0,ATH1,ATH2,ATH1,(L3|L2),(L3|L1),D0);
  mux4x1 mx1(ATL0,ATL1,ATL2,ATL1,(L3|L2),(L3|L1),D1);
 
endmodule

module test_select;
  reg EN,L0,L1,L2,L3,CLK,RST;
  wire [3:0]D0;
  wire [3:0]D1;
  number_set ns(D1,D0,EN,L0,L1,L2,L3,CLK,RST);

initial 
begin
  RST=0;
  CLK=0;
  #10
  RST=1;
  repeat(200)
  #5 CLK=~CLK;
end

initial 
begin
  EN=0;
  L0=1; L1=0; L2=0; L3=0;
  #10 EN=1;
  #500 L0=0; L2=1;
end
endmodule


module testmux;
  reg[3:0]i0;
  reg[3:0]i1;
  reg[3:0]i2;
  reg[3:0]i3;
  reg[2:0]select;
  mux4x1 mm(i0,i1,i2,i3,select[1],select[0],y);
  
  initial
  begin 
    i0=4'b0000;
     i1=4'b0100;
      i2=4'b1000;
       i3=4'b0001;
       select=2'b00;
       #10 select=2'b11;
     end
   endmodule
  



module Moore_main(A,B,CLK,finished,AL,BL,D,RST);
  
  input A,B,finished,RST,CLK;
  output reg AL,BL,D;
  
  reg  [1:0]state;
  parameter S0 = 2'b00, S1 = 2'b01, S2 = 2'b10, S3 = 2'b11;
  
   always @ (posedge CLK or negedge RST)
      if (~RST) state = S0;  //Initialize to state S0
      
   
       else      //Determine next state
         case (state)
            S0: if (finished & ~A & ~B) state = S1;
                    else if(B & ~A) state = S2;
            S1: if (finished & ~A & ~B) state = S2;  
                    else if(A)  state = S0;
            S2: if (finished & ~A & ~B) state = S3;
                    else if(A)  state = S0;
            S3: if ((finished & ~A & ~B)| A) state = S0;  
                    else if(B & ~A) state = S2;
         endcase
  
   always @ (state )     //Evaluate output
         case (state)
            S0: #1 {AL,BL,D}=101;
            S1: #1 {AL,BL,D}=001;
            S2: #1 {AL,BL,D}=010;
            S3: #1 {AL,BL,D}=000;
         endcase
endmodule



  
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
 module main(A,B,RST,CLK,L,BL,AL,finished);
   
    
   input  A,B,RST,CLK;
   output  [3:0] L;
   output  finished,AL,BL; 

   wire load,D;
   assign load = (A | B | finished);
   
   wire [7:0] O;
   wire [7:0] P;
   wire [7:0] Q;
   
 Meally_main meally(A,B,CLK,finished,AL,BL,D,RST); 
 
 
   assign  L[0]= (AL & ~BL);
   assign  L[1]= (~AL & ~BL&D);
   assign  L[2]= (~AL &  BL);
   assign  L[3]= (~AL & ~BL&~D);
   
   counter8 light90(8'b00000000,CLK,RST,load,~(A|B) & L[0],O);
   counter8 light30(8'b00000000,CLK,RST,load,~(A|B) & L[2],P);
   counter8 light5 (8'b00000000,CLK,RST,load,~(A|B) & (L[1]|L[3]),Q);
   
   wire Oa;
   assign Oa = ~O[7]&O[6]&~O[5]&O[4]&O[3]&~O[2]&O[1]&~O[0];
   wire  Pa;
   assign Pa=~P[7]& ~P[6]& ~P[5]& P[4]& P[3]& P[2]& P[1]& ~P[0] ;
   wire Qa;
   assign Qa=~Q[7]&~Q[6]&~Q[5]&~Q[4]&~Q[3]&Q[2]&~Q[1]&Q[0];
      
  assign finished = Oa| Pa | Qa;

 endmodule  
 
 
 
  module test_main ;
    
   reg A , B , RST,CLK;
   wire AL,BL,finished;
   wire [3:0] L;
  
   main my_main (A,B,RST,CLK,L,BL,AL,finished);
   
  
    initial 
     begin
      RST=0;
      CLK=0;
      #10
      RST=1;
      
      repeat(400)
      #2  
      CLK=~CLK;
    
    end
  
    initial 
      begin
      
       A=0;
       B=0;
  
      end
    
 endmodule
  
  module mux2x1(A,B,select,OUT);
   input A,B,select;
   output OUT;
   reg OUT;    
   always @ (select or A or B) 
         if (select == 1) OUT = B;
         else OUT = A;
endmodule
  
  module mux4x2(A1,A2,B1,B2,select,out1,out2);
   
   input A1,A2,B1,B2,select;
   output out1,out2;
   
   mux2x1 first(A1,A2,select,out1);
   mux2x1 second(B1,B2,select,out2);
   
endmodule

module traffic_light(A,B,R,AT,BT,CLK,ATH,ATL,AL,BL,finished1,finished2);
  input A,B,R,AT,BT,CLK;
  output AL,BL,finished1,finished2;
  output [3:0]ATH;
  output [3:0]ATL;
  wire [3:0]L;
  wire [3:0]ATL2;
  wire [3:0]ATH2;
  wire PO,S,e,AL1,BL1,AL2,BL2,EN;
  assign EN=~(PO|e);
  assign PO=A|B|R;
  assign S=e&~PO;
  mux4x2 mx(AL1,AL2,BL1,BL2,S,AL,BL);
  main m(A|e,B,~R,CLK,L,BL1,AL1,finished1);
  number_set n(ATL2,ATH2,EN,L[0],L[1],L[2],L[3],CLK,~R);
  blink b(finished2,AL2,BL2,e,PO,AT,BT,CLK,~R);  
  assign ATL[0]=PO|ATL2[0];
  assign ATH[0]=PO|ATH2[0];
  assign ATL[1]=PO|ATL2[1];
  assign ATH[1]=PO|ATH2[1];
  assign ATL[2]=PO|ATL2[2];
  assign ATH[2]=PO|ATH2[2];
  assign ATL[3]=PO|ATL2[3];
  assign ATH[3]=PO|ATH2[3];
endmodule


module test_traffic;
  reg A,B,R,AT,BT,CLK;
  wire AL,BL,finished1,finished2;
  wire[3:0] ATH;
  wire [3:0] ATL;
  traffic_light tl(A,B,R,AT,BT,CLK,ATH,ATL,AL,BL,finished1,finished2);
  initial 
  begin
    R=1;
    CLK=0;
    #15 R=0;
    repeat(4000)
    #5 CLK=~CLK;
  end
  
  initial
  begin
    A=0; B=0; AT=0; BT=0;
    #2000 AT=1; 
    #200 AT=0;
    #500 B=1;
    #200 B=0;
    #2000 BT=1;
    #1000 A=1;
    #200  A=0;
    #1000 R=1;
    #200  R=0;
    #2500 BT=0;
    #4000 AT=1;
     
    
       
  end
endmodule
  

