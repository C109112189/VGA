module work4
(
    input                      I_clk , //�t��50MHz����
    input                      I_rst_n , //�t�δ_��
    output    reg              O_red , // VGA������q
    output    reg              O_green , // VGA�����q
    output    reg              O_blue , // VGA�Ŧ���q
    output                     O_hs , // VGA��P�B�H��
    output                     O_vs       // VGA���P�B�H��
);

//����v��640*480�ɦ�ɧǦU�ӰѼƩw�q
parameter       C_H_SYNC_PULSE =    96   ,
                C_H_BACK_PORCH       =    48   ,
                C_H_ACTIVE_TIME      =    640 ,
                C_H_FRONT_PORCH      =    16   ,
                C_H_LINE_PERIOD      =    800 ;

//����v��640*480�ɳ��ɧǦU�ӰѼƩw�q               
parameter       C_V_SYNC_PULSE =    2    ,
                C_V_BACK_PORCH       =    33   ,
                C_V_ACTIVE_TIME      =    480 ,
                C_V_FRONT_PORCH      =    10   ,
                C_V_FRAME_PERIOD     =    525 ;
                
parameter       C_COLOR_BAR_WIDTH = C_H_ACTIVE_TIME / 8   ;  

reg [ 11 : 0 ] R_h_cnt ; //��ɧǭp�ƾ�
reg [ 11 : 0 ] R_v_cnt ; //�C�ɧǭp�ƾ�
reg            R_clk_50M ;
reg            R_clk_25M ;

wire           W_active_flag ; //�E���лx�A��o�ӫH����1��RGB���ƾڥi�H��ܦb�̹��W

//////////////////////////////////////////////////////////////////
 //�\��G ����25MHz����������
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
 //�\��G���ͦ�ɧ�
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
 //�\��G���ͳ��ɧ�
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
 //////////////////////////////////////////////////////////////////  

assign W_active_flag =  (R_h_cnt >= (C_H_SYNC_PULSE + C_H_BACK_PORCH )) && 
                        (R_h_cnt <= (C_H_SYNC_PULSE + C_H_BACK_PORCH + C_H_ACTIVE_TIME)) &&  
                        (R_v_cnt >= (C_V_SYNC_PULSE + C_V_BACK_PORCH )) && 
                        (R_v_cnt <= (C_V_SYNC_PULSE + C_V_BACK_PORCH + C_V_ACTIVE_TIME)) ;                     

//////////////////////////////////////////////////////////////////
 //�\��G����ܾ��̹�����8���a�C�A�C���a�C���e�׬O80 
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
            if(R_h_cnt < (C_H_SYNC_PULSE + C_H_BACK_PORCH + C_COLOR_BAR_WIDTH)) //����m��
                begin 
                    O_red    <=   1'b1 ; // ����m���������q������1�A���M�Ŧ⵹0 
                    O_green  <=   1'b0 ; 
                    O_blue   <=   1'b0 ; 
                end 
            else  if (R_h_cnt < (C_H_SYNC_PULSE + C_H_BACK_PORCH + C_COLOR_BAR_WIDTH* 2 )) //���m��
                begin 
                    O_red    <=   1'b0 ; 
                    O_green  <=   1'b1 ; // ���m��������q������1�A����M�Ŧ���q��0 
                    O_blue   <=   5'b0 ; 
                end  
            else  if (R_h_cnt < (C_H_SYNC_PULSE + C_H_BACK_PORCH + C_COLOR_BAR_WIDTH* 3 )) //�Ŧ�m��
                begin 
                    O_red    <=   1'b0 ; 
                    O_green  <=   1'b0 ; 
                    O_blue   <=   1'b1 ; // �Ŧ�m�����Ŧ���q������1�A����M����q��0
                 end  
            else  if (R_h_cnt < (C_H_SYNC_PULSE + C_H_BACK_PORCH + C_COLOR_BAR_WIDTH*4 )) //�զ�m��
                begin 
                    O_red    <=   1'b1 ; // �զ�m���O�������ŤT���V�X�Ӧ�
                    O_green  <=   1'b1 ; // �ҥH�զ�m���n������ŤT�Ӥ��q������1 
                    O_blue   <=   1'b1 ; 
                end  
            else  if (R_h_cnt < (C_H_SYNC_PULSE + C_H_BACK_PORCH + C_COLOR_BAR_WIDTH* 5 )) //�¦�m��
                begin 
                    O_red    <=   1'b0 ; // �¦�m���N�O������ũҦ����q������0 
                    O_green  <=   1'b0 ; 
                    O_blue   <=   1'b0 ; 
                end  
            else  if (R_h_cnt < (C_H_SYNC_PULSE + C_H_BACK_PORCH + C_COLOR_BAR_WIDTH* 6 )) //����m��
                begin 
                    O_red    <=   1'b1 ; // ����m���O���������C��V�X�Ӧ�
                    O_green  <=   1'b1 ; // �ҥH����m���n������Ӥ��q��1 
                    O_blue   <=   1'b0 ; // �Ŧ���q��0
                 end  
            else  if(R_h_cnt < (C_H_SYNC_PULSE + C_H_BACK_PORCH + C_COLOR_BAR_WIDTH* 7 )) //����m��
                begin 
                    O_red    <=   1'b1 ; // ����m���O�����Ũ���C��V�X�Ӧ�
                    O_green  <=   1'b0 ; // �ҥH����m���n����Ũ�Ӥ��q��1 
                    O_blue   <=   1'b1 ; // �����q��0
                 end  
            else                               //�C��m��
                begin 
                    O_red    <=   1'b0 ; // �C��m���O���ź����C��V�X�Ӧ�
                    O_green  <=   1'b1 ; // �ҥH�C��m���n���ź��Ӥ��q��1 
                    O_blue   <=   1'b1 ; // ������q��0
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
