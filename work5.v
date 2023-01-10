`timescale 1ns / 1ps
module work5
(
    input                      clk , //系統100MHz時鐘
    input                      rst, //系統復位
    input                      button0,//打者一
    input                      button1,//打者二
    input                      sw,//分數顯示器
    output                     [7:0]led,//led顯示
    output                     red , // VGA紅色分量
    output                     green , // VGA綠色分量
    output                     blue , // VGA藍色分量
    output                     hs , // VGA行同步信號
    output                     vs       // VGA場同步信號
 );

wire b0,b1;
wire divclk_1,divclk_2;
wire [3:0] Rs,Ls;    

div div1(divclk_1,divclk_2,clk,rst);
button bt0(b0,button0,clk,rst);
button bt1(b1,button1,clk,rst);
FSM FSM1(divclk_2,rst,b0,b1,led,Mstar,flag,ledc,Rs,Ls);
MRL MRL1(divclk_1,rst,led,flag,Mstar,ledc,Rs,Ls,sw);  
VGA VGA1(clk,rst,led,red,green,blue,hs,vs);

endmodule

module FSM(clk,rst,button0,button1,led,Mstar,flag,ledc,Rs,Ls);
	
	input clk,rst;
	input button0,button1;
	input [7:0] led;
	output reg Mstar,flag,ledc;
	output reg [3:0] Rs,Ls;
//	output reg [7:0] seg7_out;
	reg[2:0] state;
	
	parameter Lstar=3'd0 , Rstar=3'd1 , MR=3'd2 , ML=3'd3; 
	
	always@(posedge clk or negedge rst)
		begin
			if(rst)
				begin
					state<=Lstar;
//					seg7_out<=8'b0;
					Mstar<=0;
					flag<=0;
					ledc<=0;
					Rs<=0;
					Ls<=0;
				end
			else
				begin
					case(state)
						Lstar:
							begin
								if(flag==0 && button0==1)
									begin
										Mstar<=1;
										state<=MR;
										ledc<=1;
									end
								else
									begin
										Mstar<=0;
										state<=Lstar;
										ledc<=0;
									end
							end
						Rstar:
							begin
								if(flag==1 && button1==1)
									begin
										Mstar<=1;
										state<=ML;
										ledc<=1;
									end
								else
									begin
										Mstar<=0;
										state<=Rstar;
										ledc<=0;
									end
							end
						MR:
							begin
							    if(button1==1 && led==8'b0000_0001)
							        begin
							            state<=ML;
							            flag<=1;
							        end
								else if((button1==1 && led!=8'b0000_0001) || led==8'b0000_0000)
									begin
										state<=Lstar;
										Mstar<=0;
										ledc<=0;
										Ls<=Ls+1;
									end
								else
									state<=MR;
							end
						ML:
							begin
							    if(button0==1 && led==8'b1000_0000)
							        begin
							            state<=MR;
							            flag<=0;
							        end
								else if((button0==1 && led!=8'b1000_0000) || led==8'b0000_0000)
									begin
										state<=Rstar;
										Mstar<=0;
										ledc<=0;
										Rs<=Rs+1;
									end
								else
									state<=ML;
							end
					endcase				
				end
		end
endmodule
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
module MRL(clk,rst,led,flag,Mstar,ledc,Rs,Ls,sw);
	
	input clk,rst;
	input flag,Mstar,ledc,sw;
	input [3:0] Rs,Ls;
	output reg [7:0] led;
	reg [3:0] SR,SL;

	
	always@(posedge clk or negedge rst)
		begin
			if(rst)
			    begin
				    led<=8'b1000_0000;
				    SR<=0;
				    SL<=0;
				end
			else 
				begin
				    SL<=Ls;
				    SR<=Rs;
				    if(sw==1)
				        begin
				            led<={SL,SR};
				        end
				    else
				        begin
					       if(flag==0 && ledc==0)
					           led<=8'b1000_0000;
					       if(flag==1 && ledc==0)
						      led<=8'b0000_0001;	
					       if(flag==0 && ledc==1)
						      led<=led/2;
					       if(flag==1 && ledc==1)
						      led<=led*2;
						end
				end
		end
endmodule
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
module button(click,in,clk,rst);
	output reg click;
	input in,clk,rst;
	reg [23:0]decnt;

	parameter bound = 24'h000f0f;

	always @ (posedge clk or negedge rst)begin
		if(rst)begin
			decnt <= 0;
			click <= 0;
		end
		else begin
			if(in)begin
				if(decnt < bound)begin
					decnt <= decnt + 1;
					click <= 0;
				end
				else begin
					decnt <= decnt;
					click <= 1;
				end
			end
			else begin
				decnt <= 0;
				click <= 0;
			end
		end
	end
endmodule
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
module div(divclk_1,divclk_2,clk,rst);

input clk,rst;
output divclk_1,divclk_2;
reg [27:0]divclkcnt;

assign divclk_1 = divclkcnt[25];
assign divclk_2 = divclkcnt[20];

always@(posedge clk or negedge rst)begin
    if(rst)
        divclkcnt = 0;
    else
        divclkcnt = divclkcnt + 1;
end
endmodule

//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
module VGA
(
    input                      I_clk , //系統50MHz時鐘
    input                      I_rst_n , //系統復位
    input                      [7:0] led,//輸入辨識用led訊號
    output    reg              O_red , // VGA紅色分量
    output    reg              O_green , // VGA綠色分量
    output    reg              O_blue , // VGA藍色分量
    output                     O_hs , // VGA行同步信號
    output                     O_vs       // VGA場同步信號
);

//分辨率為640*480時行時序各個參數定義
parameter       C_H_SYNC_PULSE =    96   ,
                C_H_BACK_PORCH       =    48   ,
                C_H_ACTIVE_TIME      =    640 ,
                C_H_FRONT_PORCH      =    16   ,
                C_H_LINE_PERIOD      =    800 ;

//分辨率為640*480時場時序各個參數定義               
parameter       C_V_SYNC_PULSE =    2    ,
                C_V_BACK_PORCH       =    33   ,
                C_V_ACTIVE_TIME      =    480 ,
                C_V_FRONT_PORCH      =    10   ,
                C_V_FRAME_PERIOD     =    525 ;
                
parameter       C_COLOR_BAR_WIDTH = C_H_ACTIVE_TIME / 8      ;  
parameter       C_COLOR_BAR__V_WIDTH = C_V_ACTIVE_TIME / 5   ;  


reg [ 11 : 0 ] R_h_cnt ; //行時序計數器
reg [ 11 : 0 ] R_v_cnt ; //列時序計數器
reg            R_clk_50M ;
reg            R_clk_25M ;
reg [7:0]      vgaled;

wire           W_active_flag ; //激活標誌，當這個信號為1時RGB的數據可以顯示在屏幕上

//////////////////////////////////////////////////////////////////
 //功能： 產生25MHz的像素時鐘
//////////////////////////////////////////////////////////////////
 always @( posedge I_clk or  negedge I_rst_n)
 begin 
    if ( I_rst_n)
        R_clk_50M    <=   1'b0 ; 
    else 
        R_clk_50M    <= ~ R_clk_50M ;     
 end 
//////////////////////////////////////////////////////////////////
always @( posedge R_clk_50M or  negedge I_rst_n)
 begin 
    if ( I_rst_n)
        R_clk_25M    <=   1'b0 ; 
    else 
        R_clk_25M    <= ~ R_clk_25M ;     
 end
//////////////////////////////////////////////////////////////////
 //功能：產生行時序
//////////////////////////////////////////////////////////////////
 always @( posedge R_clk_25M or  negedge I_rst_n)
 begin 
    if ( I_rst_n)
        R_h_cnt <=   12'd0 ; 
    else  if (R_h_cnt == C_H_LINE_PERIOD - 1'b1) 
        R_h_cnt <=   12'd0 ; 
    else 
        R_h_cnt <= R_h_cnt + 1'b1 ;                 
end                

assign O_hs = (R_h_cnt < C_H_SYNC_PULSE) ? 1'b0 : 1'b1 ; 
 //////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
 //功能：產生場時序
//////////////////////////////////////////////////////////////////
 always @( posedge R_clk_25M or  negedge I_rst_n)
 begin 
    if ( I_rst_n)
        R_v_cnt <=   12'd0 ; 
    else  if (R_v_cnt == C_V_FRAME_PERIOD - 1'b1) 
        R_v_cnt <=   12'd0 ; 
    else  if (R_h_cnt == C_H_LINE_PERIOD - 1'b1) 
        R_v_cnt <= R_v_cnt + 1'b1 ; 
    else 
        R_v_cnt <=   R_v_cnt ;                        
 end                

assign O_vs = (R_v_cnt < C_V_SYNC_PULSE) ? 1'b0 : 1'b1 ; 

always @( posedge R_clk_25M or  negedge I_rst_n)
begin
    if(I_rst_n)
      vgaled <= 0;
    else if(led[0]==1)
        vgaled<=0;
    else if(led[1]==1)
        vgaled<=1;
    else if(led[2]==1)
        vgaled<=2;
    else if(led[3]==1)
        vgaled<=3;
    else if(led[4]==1)
        vgaled<=4;
    else if(led[5]==1)
        vgaled<=5;
    else if(led[6]==1)
        vgaled<=6;
    else if(led[7]==1)
        vgaled<=7;
end

 //////////////////////////////////////////////////////////////////  
assign W_active_flag  =  (R_h_cnt >= (C_H_SYNC_PULSE + C_H_BACK_PORCH + C_COLOR_BAR_WIDTH*vgaled)) && 
                         (R_h_cnt <= (C_H_SYNC_PULSE + C_H_BACK_PORCH + C_COLOR_BAR_WIDTH*(vgaled+1))) &&  
                         (R_v_cnt >= (C_V_SYNC_PULSE + C_V_BACK_PORCH + C_COLOR_BAR__V_WIDTH*2)) && 
                         (R_v_cnt <= (C_V_SYNC_PULSE + C_V_BACK_PORCH + C_COLOR_BAR__V_WIDTH*3)) ;  
//////////////////////////////////////////////////////////////////
 //功能：把顯示器屏幕分成8個縱列，每個縱列的寬度是80 
//////////////////////////////////////////////////////////////////
 always @( posedge R_clk_25M or  negedge I_rst_n)
 begin 
    if ( I_rst_n) 
         begin 
            O_red    <=   1'b0 ; 
            O_green  <=   1'b0 ; 
            O_blue   <=   1'b0 ; 
        end 
    else  if (W_active_flag)     
         begin 
            if(R_h_cnt < (C_H_SYNC_PULSE + C_H_BACK_PORCH + C_COLOR_BAR_WIDTH) )
                begin 
                            O_red    <=   1'b1 ; // 紅色彩條把紅色分量全部給1，綠色和藍色給0 
                            O_green  <=   1'b0 ; 
                            O_blue   <=   1'b0 ; 
                end 
            else  if (R_h_cnt < (C_H_SYNC_PULSE + C_H_BACK_PORCH + C_COLOR_BAR_WIDTH* 2 ))
                begin
                            O_red    <=   1'b0 ; 
                            O_green  <=   1'b1 ; // 綠色彩條把綠色分量全部給1，紅色和藍色分量給0 
                            O_blue   <=   5'b0 ; 
                end  
            else  if (R_h_cnt < (C_H_SYNC_PULSE + C_H_BACK_PORCH + C_COLOR_BAR_WIDTH* 3 ))
                    begin
                                O_red    <=   1'b0 ; 
                                O_green  <=   1'b0 ; 
                                O_blue   <=   1'b1 ; // 藍色彩條把藍色分量全部給1，紅色和綠分量給0
                    end
            else  if (R_h_cnt < (C_H_SYNC_PULSE + C_H_BACK_PORCH + C_COLOR_BAR_WIDTH*4 ))
                    begin
                                 O_red    <=   1'b1 ; // 白色彩條是有紅綠藍三基色混合而成
                                 O_green  <=   1'b1 ; // 所以白色彩條要把紅綠藍三個分量全部給1 
                                 O_blue   <=   1'b1 ; 
                end  
            else  if (R_h_cnt < (C_H_SYNC_PULSE + C_H_BACK_PORCH + C_COLOR_BAR_WIDTH* 5 ))
                    begin
                    O_red    <=   1'b0 ; // 黑色彩條就是把紅綠藍所有分量全部給0 
                    O_green  <=   1'b0 ; 
                    O_blue   <=   1'b0 ; 
                end  
            else  if (R_h_cnt < (C_H_SYNC_PULSE + C_H_BACK_PORCH + C_COLOR_BAR_WIDTH* 6 ))
            begin
                    O_red    <=   1'b1 ; // 黃色彩條是有紅綠兩種顏色混合而成
                    O_green  <=   1'b1 ; // 所以黃色彩條要把紅綠兩個分量給1 
                    O_blue   <=   1'b0 ; // 藍色分量給0
                 end  
            else  if(R_h_cnt < (C_H_SYNC_PULSE + C_H_BACK_PORCH + C_COLOR_BAR_WIDTH* 7 ))
                begin
                    O_red    <=   1'b1 ; // 紫色彩條是有紅藍兩種顏色混合而成
                    O_green  <=   1'b0 ; // 所以紫色彩條要把紅藍兩個分量給1 
                    O_blue   <=   1'b1 ; // 綠色分量給0
                 end  
            else                               //青色彩條
                begin 
                    O_red    <=   1'b0 ; // 青色彩條是由藍綠兩種顏色混合而成
                    O_green  <=   1'b1 ; // 所以青色彩條要把藍綠兩個分量給1 
                    O_blue   <=   1'b1 ; // 紅色分量給0
                 end                    
        end 
    else 
        begin 
            O_red    <=   1'b0 ; 
            O_green  <=   1'b0 ; 
            O_blue   <=   1'b0 ; 
        end            
end 

endmodule