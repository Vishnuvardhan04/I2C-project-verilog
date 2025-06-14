module piso_address(clock,load,shift,address,rd_wr,serial_out);
input clock,load,shift,rd_wr;
input [6:0]address;
output serial_out;
reg [7:0]temp;

always@(negedge clock)begin
if(load)
temp<={address,rd_wr};
else if(shift)
temp<={temp[6:0],1'b0};
end

assign serial_out=temp[7];

endmodule

/*module piso_address_tb;
reg clock=0;
reg load,shift,rd_wr;
reg [6:0]address;
wire serial_out;
piso_address p1(clock,load,shift,address,rd_wr,serial_out);

always #5 clock=~clock;

initial begin
    address=7'b1100110;
    rd_wr=1;
    load=1;
    #6 load=0;
    #6 shift=1;
end
endmodule*/

module mast_piso_data(master_data, master_scl_sixt, master_load_data, master_shift_data, master_serial_out_data);
input [7:0] master_data;
input master_shift_data,master_load_data;
input master_scl_sixt;
output master_serial_out_data;
reg [7:0] master_temp_data;

always@(negedge master_scl_sixt) begin
if(master_load_data)
master_temp_data<=master_data;
else if(master_shift_data)
master_temp_data<={master_temp_data[6:0],1'b0};
end

assign master_serial_out_data=master_temp_data[7];
endmodule

/*module piso_address_tb;
reg clock=0;
reg load,shift;
reg [7:0]data;
wire serial_out;
mast_piso_data p1(data,clock,load,shift,serial_out);

always #5 clock=~clock;

initial begin
    data=8'b11001101;
    load=0;
    #2 load=1;
    #9 shift=1; load=0;
end
endmodule
*/

 module mast_sipo_slave_data(master_serial_in, master_scl_sixt, master_rec_data_shift, master_data_out);
 input master_serial_in,master_rec_data_shift;
 input master_scl_sixt;
 output [7:0]master_data_out;
 reg [7:0] master_temp_received;

 always@(posedge master_scl_sixt)begin
 if(master_rec_data_shift)
 master_temp_received<={master_temp_received[6:0],master_serial_in};
 end

 //assign master_data_out=master_temp_received;
assign master_data_out=master_rec_data_shift?master_data_out:master_temp_received;
 endmodule

/*module mast_sipo_slave_data_tb;
reg master_serial_in,master_rec_data_shift;
reg master_scl_sixt=0;
wire [7:0]master_data_out;
mast_sipo_slave_data m1(master_serial_in,master_scl_sixt,master_rec_data_shift,master_data_out);

always #5 master_scl_sixt=~master_scl_sixt;

initial begin
    master_serial_in=0;
    master_rec_data_shift=1;
    #13 master_serial_in=0;
    #10 master_serial_in=0;
    #10 master_serial_in=0;
    #10 master_serial_in=1;
    #10 master_serial_in=1;
    #10 master_serial_in=1;
    #10 master_serial_in=1;
    #10 master_rec_data_shift=0;
end
endmodule
*/



 module mast_tristate_logic(master_sda_out, master_tri_en, master_sda_in, master_sda );
 input master_sda_out,master_tri_en;
 output master_sda_in;
 inout master_sda;

 bufif0  b1(master_sda, master_sda_out, master_tri_en);
 buf b2(master_sda_in, master_sda);

 endmodule

/*module mast_traistate_logic_tb;
reg master_sda_out,master_tri_en;
wire master_sda_in;
wire master_sda;
mast_tristate_logic t1(master_sda_out, master_tri_en, master_sda_in, master_sda);

initial begin
    master_sda_out=1;
    master_tri_en=0;
    #10 master_sda_out=0;
    #10 master_tri_en=1;
end
endmodule*/


module scl_buf(master_scl, mast_scl_out,mast_scl_en);
input mast_scl_en;
input master_scl;
output mast_scl_out;

bufif1 buf11(mast_scl_out,master_scl,mast_scl_en);

endmodule

module mux_4x1(address,data,ack,mux_sel,mux_out);
input [1:0]mux_sel;
input address,data,ack;
output reg mux_out;

always@(mux_sel,address,data,ack)begin
case(mux_sel)
2'b00: mux_out=0;
2'b01: mux_out=address;
2'b10: mux_out=data;
2'b11: mux_out=ack;
endcase
end
endmodule

module mux_2x1(ack_sel,ack_out);
input ack_sel;
output reg ack_out;

parameter ACK = 0, NACK = 1;
always@(ack_sel)begin
case(ack_sel)
1'b0:ack_out=NACK;
1'b1:ack_out=ACK;
endcase
end
endmodule

module mast_demux(master_demux_select,master_demux_in,master_slavedata,master_ack);
input master_demux_select,master_demux_in;
output reg master_slavedata,master_ack;

always@(master_demux_select,master_demux_in)
case(master_demux_select)
1'b0:begin
     master_ack=master_demux_in;
     master_slavedata=1'b0;
end
1'b1:begin
     master_ack=1'b0;
     master_slavedata=master_demux_in;
end
endcase
endmodule

/*module mast_ack(master_scl_sixt, master_ack_sel, master_ack_out);
input master_scl_sixt;
input master_ack_sel;
output master_ack_out;
parameter ACK=1'b0, NACK=1'b1;

always@(*)
if(master_ack_sel)
master_ack_out=ACK;
else
master_ack_out=NACK;
endmodule*/



module mast_fsm(master_start_bit, master_rd_wr, master_scl_sixt, master_rst, master_ack, master_load_add, master_shift_add,master_load_data, master_shift_data, master_mux_sel, master_tri_en, master_demux_sel, master_shift_d_slave, master_ack_sel, master_scl_en);
input master_start_bit,master_rd_wr,master_rst,master_ack,master_scl_sixt;
output reg master_load_add,master_shift_add;
output reg master_load_data,master_shift_data,master_ack_sel;
output reg master_tri_en,master_demux_sel,master_shift_d_slave;
output reg master_scl_en;
output reg[1:0]master_mux_sel;

parameter master_idle=4'b0000,master_start=4'b0001,transmit_scl=4'b0010,master_address=4'b0011,master_ack_add=4'b0100,master_send_data=4'b0101,master_ack_rd=4'b0110,
          master_receive_data=4'b0111,master_ack_sd=4'b1000,master_stop=4'b1001;

reg [3:0]master_state;
reg [3:0]master_next_state;
reg go;
integer master_count=1;

always@(negedge master_scl_sixt)begin
if(go)
master_count<=master_count+1;
else
master_count<=1;
end

always@(master_state,master_rd_wr,master_ack,master_start_bit,master_count)
case(master_state)
master_idle:
master_next_state=(master_start_bit==1)?master_start:master_idle;

master_start:
master_next_state=transmit_scl;

transmit_scl:
master_next_state=master_address;

master_address:begin
master_next_state=(master_count==8)?master_ack_add:master_address;
go=(master_count==8)?0:1;end

master_ack_add:
master_next_state=(master_ack==0)?((master_rd_wr==0)?master_send_data:master_receive_data):master_idle;

master_send_data:begin
master_next_state=(master_count==8)?master_ack_rd:master_send_data;
go=(master_count==8)?0:1;end

master_ack_rd:
master_next_state=(master_ack==0)?master_stop:master_send_data;

master_receive_data:begin
master_next_state=(master_count==8)?master_ack_sd:master_receive_data;
go=(master_count==8)?0:1;end

master_ack_sd:
master_next_state=master_stop;

master_stop:
master_next_state=master_idle;

default:
master_next_state=master_idle;

endcase

always@(negedge master_scl_sixt,negedge master_rst)
if(!master_rst)
master_state<=master_idle;
else
master_state=master_next_state;

always@(master_state)begin
if(master_state==master_idle)begin
master_tri_en=1'b1;          master_demux_sel=1'b0;         master_shift_d_slave=1'b0;
master_load_add=1'b0;        master_shift_add=1'b0;         master_ack_sel=0;
master_load_data=1'b0;       master_shift_data=1'b0;         master_scl_en=0;
end

else if(master_state==master_start)begin
master_tri_en=1'b0;          master_demux_sel=1'b0;         master_shift_d_slave=1'b0;
master_load_add=1'b1;        master_shift_add=1'b0;         master_ack_sel=0;
master_load_data=1'b0;       master_shift_data=1'b0;        master_mux_sel=2'b00;
master_scl_en=0;
end

else if(master_state==transmit_scl)begin
master_tri_en=1'b0;          master_demux_sel=1'b0;         master_shift_d_slave=1'b0;
master_load_add=1'b0;        master_shift_add=1'b0;         master_ack_sel=0;
master_load_data=1'b0;       master_shift_data=1'b0;        master_mux_sel=2'b00;
master_scl_en=1;
end

else if(master_state==master_address)begin
master_tri_en=1'b0;          master_demux_sel=1'b0;         master_shift_d_slave=1'b0;
master_load_add=1'b0;        master_shift_add=1'b1;         master_ack_sel=0;
master_load_data=1'b0;       master_shift_data=1'b0;        master_mux_sel=2'b01;
master_scl_en=1;
end

else if(master_state==master_ack_add)begin
master_tri_en=1'b1;          master_demux_sel=1'b0;         master_shift_d_slave=1'b0;
master_load_add=1'b0;        master_shift_add=1'b0;         master_ack_sel=0;
master_load_data=1'b1;       master_shift_data=1'b0;        master_mux_sel=2'b00;
master_scl_en=1;
end

else if(master_state==master_send_data)begin
master_tri_en=1'b0;          master_demux_sel=1'b0;         master_shift_d_slave=1'b0;
master_load_add=1'b0;        master_shift_add=1'b0;         master_ack_sel=0;
master_load_data=1'b0;       master_shift_data=1'b1;        master_mux_sel=2'b10;
master_scl_en=1;
end

else if(master_state==master_receive_data)begin
master_tri_en=1'b1;          master_demux_sel=1'b1;         master_shift_d_slave=1'b1;
master_load_add=1'b0;        master_shift_add=1'b0;         master_ack_sel=0;
master_load_data=1'b0;       master_shift_data=1'b0;        master_mux_sel=2'b00;
master_scl_en=1;
end

else if(master_state==master_ack_rd)begin
master_tri_en=1'b1;          master_demux_sel=1'b0;         master_shift_d_slave=1'b0;
master_load_add=1'b0;        master_shift_add=1'b0;         master_ack_sel=0;
master_load_data=1'b0;       master_shift_data=1'b0;        master_mux_sel=2'b11;
master_scl_en=1;
end

else if(master_state==master_ack_sd)begin
master_tri_en=1'b0;          master_demux_sel=1'b0;         master_shift_d_slave=1'b0;
master_load_add=1'b0;        master_shift_add=1'b0;         master_ack_sel=1;
master_load_data=1'b0;       master_shift_data=1'b0;        master_mux_sel=2'b11;
master_scl_en=1;
end

else if(master_state==master_stop)begin
master_tri_en=1'b1;          master_demux_sel=1'b0;         master_shift_d_slave=1'b0;
master_load_add=1'b0;        master_shift_add=1'b0;         master_ack_sel=0;
master_load_data=1'b0;       master_shift_data=1'b0;        master_mux_sel=2'b11;
master_scl_en=0;
end

else
begin
master_tri_en=1'b0;          master_demux_sel=1'b0;         master_shift_d_slave=1'b0;
master_load_add=1'b0;        master_shift_add=1'b0;         master_ack_sel=0;
master_load_data=1'b0;       master_shift_data=1'b0;        master_mux_sel=2'b11;
master_scl_en=0;
end
end

endmodule

module simple_master(mast_start_bit,fpga_clk,mast_rd_wr,mast_address,mast_data,mast_rst,scl,sda,data_from_slave);
input mast_start_bit,mast_rd_wr;
input fpga_clk,mast_rst;
input [6:0]mast_address;
input [7:0]mast_data;
output [7:0]data_from_slave;
inout tri1 sda;
output tri1 scl;
wire mast_load_addr,mast_shift_add,mast_serial_out_addr;
wire mast_load_data,mast_shift_data,mast_serial_out_data,mast_shift_d_slave;
wire mast_scl;
wire mast_tri_en,mast_sda_in,mast_sda_out,mast_demux_sel,mast_ack,mast_slavedata;
wire mast_scl_en,mast_ack_sel,mast_ack_out;
wire[1:0]mast_mux_sel;

piso_address p1(.rd_wr(mast_rd_wr),.address(mast_address),.load(mast_load_addr),.shift(mast_shift_add),.clock(fpga_clk),.serial_out(mast_serial_out_addr));
mast_piso_data p2(.master_data(mast_data), .master_scl_sixt(fpga_clk),.master_load_data(mast_load_data),.master_shift_data(mast_shift_data),.master_serial_out_data(mast_serial_out_data));
mast_sipo_slave_data p3(.master_serial_in(mast_slavedata),.master_scl_sixt(fpga_clk),.master_rec_data_shift(mast_shift_d_slave),.master_data_out(data_from_slave));
mast_tristate_logic t1(.master_sda_out(mast_sda_out),.master_tri_en(mast_tri_en),.master_sda_in(mast_sda_in),.master_sda(sda));
scl_buf b1(.master_scl(fpga_clk),.mast_scl_out(mast_scl),.mast_scl_en(mast_scl_en));
mux_4x1 m1(.address(mast_serial_out_addr),.data(mast_serial_out_data),.ack(mast_ack_out),.mux_sel(mast_mux_sel),.mux_out(mast_sda_out));
mux_2x1 m2(.ack_sel(mast_ack_sel),.ack_out(mast_ack_out));
mast_demux d1(.master_demux_select(mast_demux_sel),.master_demux_in(mast_sda_in),.master_slavedata(mast_slavedata),.master_ack(mast_ack));
mast_fsm f1(.master_start_bit(mast_start_bit),.master_rd_wr(mast_rd_wr),.master_scl_sixt(fpga_clk),.master_rst(mast_rst),.master_ack(mast_ack),.master_load_add(mast_load_addr),.master_shift_add(mast_shift_add),
                                        .master_load_data(mast_load_data),.master_shift_data(mast_shift_data),.master_mux_sel(mast_mux_sel),.master_tri_en(mast_tri_en),.master_demux_sel(mast_demux_sel),.master_shift_d_slave(mast_shift_d_slave),.master_ack_sel(mast_ack_sel),.master_scl_en(mast_scl_en));
assign scl=mast_scl;
endmodule

module simple_master_tb;
reg mast_start_bit,mast_rd_wr;
reg fpga_clk=0;
reg mast_rst;
reg [6:0]mast_address;
reg [7:0]mast_data;
wire [7:0]data_from_slave;
wire sda;
wire scl;

simple_master s1(mast_start_bit,fpga_clk,mast_rd_wr,mast_address,mast_data,mast_rst,scl,sda,data_from_slave);

always #4 fpga_clk=~fpga_clk;

initial begin
mast_rst=0;mast_start_bit=1;
#3 mast_rst=1;
#2000 mast_start_bit=0;
end

initial begin
mast_address=7'b1111010; mast_rd_wr=0;
mast_data =8'b10010101;
end
endmodule


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module slave_start_stop_detector(slave_reset,slave_scl_in,slave_sda_in,start_stop_detect);
input slave_reset;
input slave_scl_in,slave_sda_in;
output start_stop_detect;
//wire rst_clock;
wire t,s1,s2;

slave_toggle_register s13(.in(slave_scl_in),.out(t),.rst(slave_reset),.clk(slave_sda_in));
slave_reg1 s23(.in(t),.out(s1),.rst(slave_reset),.clk(slave_sda_in),.load(slave_scl_in));
slave_reg2 s33(.in(t),.out(s2),.rst(slave_reset),.clk(slave_sda_in),.load(slave_scl_in));

assign start_stop_detect= ~(s1^s2);

endmodule

module slave_toggle_register(in,out,rst,clk);
input in,rst;
input clk;
output reg out;

always@(posedge clk,negedge rst)begin
if(!rst)
out<=1;
else if(in)
out<=!out;
end
endmodule

module slave_reg1(in,out,rst,clk,load);
input in,rst,clk,load;
output reg out;

always@(posedge clk,negedge rst)begin
if(!rst)
out<=0;
else if(load)
out<=in;
end
endmodule

module slave_reg2(in,out,rst,clk,load);
input in,rst,clk,load;
output reg out;

always@(negedge clk,negedge rst)begin
if(!rst)
out<=0;
else if(load)
out<=in;
end
endmodule






module slave_tristate_logic(slave_sda_out,slave_tri_en,slave_sda_in,slave_sda);
input slave_sda_out,slave_tri_en;
inout slave_sda;
output slave_sda_in;

bufif0 b1(slave_sda, slave_sda_out, slave_tri_en);
buf b2(slave_sda_in, slave_sda);

endmodule


module slave_sipo_addr(slave_serial_in, slave_scl_sixt, slave_rec_addr_shift, slave_addr_out, slave_rd_wr);
input slave_serial_in, slave_rec_addr_shift;
input slave_scl_sixt;
output [6:0]slave_addr_out;
output slave_rd_wr;
reg [7:0]slave_temp_addr;

always@(posedge slave_scl_sixt)
if(slave_rec_addr_shift)
slave_temp_addr<={slave_temp_addr[6:0], slave_serial_in};
else
slave_temp_addr<= slave_temp_addr;
assign slave_addr_out=slave_temp_addr[7:1];
assign slave_rd_wr=slave_temp_addr[0];
                    
endmodule

module slave_sipo_data(slave_serial_in,slave_rec_data_shift,slave_scl_sixt,slave_data_out);
input slave_serial_in,slave_rec_data_shift;
input slave_scl_sixt;
output [7:0]slave_data_out;
reg [7:0]slave_temp_data;

always @(posedge slave_scl_sixt)
if(slave_rec_data_shift)
slave_temp_data<= {slave_temp_data[6:0], slave_serial_in};
else
slave_temp_data<= slave_temp_data;
                        
assign slave_data_out= slave_temp_data;
                        
endmodule

module slave_piso_data(slave_data,slave_scl_sixt,slave_load_data,slave_shift_data,slave_serial_out_data);
input [7:0]slave_data;
input slave_load_data,slave_shift_data;
input slave_scl_sixt;
output slave_serial_out_data;
reg [7:0]slave_temp_data;

always@(negedge slave_scl_sixt)
if(slave_load_data)
slave_temp_data=slave_data;
else if(slave_shift_data)
slave_temp_data={slave_temp_data[6:0],1'b0};
assign slave_serial_out_data=slave_temp_data[7];

endmodule


module slave_mux(slave_mux_sel,slave_serial_out_data,slave_mux_out,slave_acknowledge);
input slave_serial_out_data,slave_acknowledge,slave_mux_sel;
output reg slave_mux_out;
//assign slave_mux_out=slave_mux_sel?slave_serial_out_data:slave_acknowledge;

always@(slave_mux_sel,slave_serial_out_data,slave_acknowledge)
if(slave_mux_sel)
slave_mux_out=slave_serial_out_data;
else
slave_mux_out=slave_acknowledge;

endmodule

module slave_mux2x1(slave_ack_sel,slave_ack_out);
input slave_ack_sel;
output reg slave_ack_out;
parameter ACK=1'b0, NACK=1'b1;

always@(*)
if(slave_ack_sel)
slave_ack_out=ACK;
else 
slave_ack_out=NACK;
endmodule

/*module slave_mux2x1_tb;
reg slave_ack_sel;
wire slave_ack_out;
slave_mux2x1 m1(slave_scl_sixt,slave_ack_sel,slave_ack_out);

initial begin
slave_ack_sel=1'b1;
#5 slave_ack_sel=1'b0;
end
endmodule*/

module slave_fsm(slave_tri_en, slave_shift_addr, slave_shift_piso,slave_load_piso, slave_shift_data, 
                                slave_ack_sel, slave_mux_sel, slave_scl_sixt, slave_ack, slave_start_bit, slave_addr_rcvd, slave_rd_wr,  slave_reset);
input slave_scl_sixt, slave_reset;
input slave_ack, slave_start_bit;
input slave_rd_wr;
input [6:0] slave_addr_rcvd;
output reg slave_tri_en, slave_ack_sel, slave_mux_sel;
output reg slave_shift_piso, slave_load_piso;
output reg slave_shift_addr,slave_shift_data;
parameter slave_device_addr= 7'b1111010;
parameter sdetect_start= 4'b0000,sstart=4'b0001, saddress_detect=4'b0010, sread_write=4'b0011, ssend_data= 4'b0100, 
                                sreceive_data =4'b0101, sack_send= 4'b0110, sack_rcvd= 4'b0111, sdetect_stop= 4'b1000;
reg [3:0]s_state;
reg [3:0]s_next_state;
reg go_slave;
integer slave_count;
reg slave_check;

always @(negedge slave_scl_sixt, negedge slave_reset)
if(!slave_reset)begin
slave_count<=1;
end
else if(go_slave)
slave_count<=slave_count+1;
else
slave_count<=1;

always@(s_state, slave_ack, slave_start_bit, slave_rd_wr, slave_addr_rcvd,slave_count)
case(s_state)
sdetect_start:s_next_state=(!slave_start_bit)?sstart:sdetect_start;

sstart:s_next_state=saddress_detect;

saddress_detect:
begin s_next_state=(slave_count==8)?sread_write:saddress_detect;
      go_slave=(slave_count==8)?0:1;
end

sread_write:
s_next_state= (slave_addr_rcvd==slave_device_addr)?(slave_rd_wr ? ssend_data: sreceive_data): sdetect_start;

ssend_data:begin
s_next_state=(slave_count==8)?sack_rcvd:ssend_data;
go_slave=(slave_count==8)?0:1;
end

sreceive_data:begin
go_slave=(slave_count==8)?0:1;
s_next_state=(slave_count==8)?sack_send:sreceive_data;
//go_slave=(slave_count==8)?0:1;
end

sack_rcvd:
s_next_state= slave_ack?sdetect_start:sdetect_stop;

sack_send:
s_next_state= sdetect_stop;

sdetect_stop:begin
s_next_state=sdetect_start;
slave_count=1;
end
endcase

always @(negedge slave_scl_sixt, negedge slave_reset)
if(!slave_reset)
s_state= sdetect_start;
else
s_state<= s_next_state;

always@(s_state, slave_rd_wr)
if(s_state==sdetect_start)
begin
slave_tri_en=1;          slave_ack_sel=0;           slave_mux_sel=0;
slave_shift_piso=0;      slave_load_piso=0;         slave_shift_data=0;
slave_shift_addr=0;
end

else if(s_state==sstart)begin
slave_tri_en=1;          slave_ack_sel=0;           slave_mux_sel=0;
slave_shift_piso=0;      slave_load_piso=0;         slave_shift_data=0;
slave_shift_addr=0;
end

else if(s_state==saddress_detect)begin
slave_tri_en=1;          slave_ack_sel=0;           slave_mux_sel=0;
slave_shift_piso=0;      slave_load_piso=0;         slave_shift_data=0;
slave_shift_addr=1;
end

else if(s_state==sread_write)begin
slave_tri_en=0;          slave_ack_sel=(slave_addr_rcvd==slave_device_addr);           slave_mux_sel=0;
slave_shift_piso=0;      slave_load_piso=slave_rd_wr?1:0;         slave_shift_data=0;
slave_shift_addr=0;
end

else if(s_state==ssend_data)begin
slave_tri_en=0;          slave_ack_sel=0;           slave_mux_sel=1;
slave_shift_piso=1;      slave_load_piso=0;         slave_shift_data=0;
slave_shift_addr=0;
end

else if(s_state==sreceive_data)begin
slave_tri_en=1;          slave_ack_sel=0;           slave_mux_sel=0;
slave_shift_piso=0;      slave_load_piso=0;         slave_shift_data=1;
slave_shift_addr=0;
end

else if(s_state==sack_send)begin
slave_tri_en=0;          slave_ack_sel=0;           slave_mux_sel=0;
slave_shift_piso=0;      slave_load_piso=0;         slave_shift_data=0;
slave_shift_addr=0;
end

else if(s_state==sack_rcvd)begin
slave_tri_en=1;          slave_ack_sel=1;           slave_mux_sel=0;
slave_shift_piso=0;      slave_load_piso=0;         slave_shift_data=0;
slave_shift_addr=0;
end

else if(s_state==sdetect_stop)begin
slave_tri_en=1;          slave_ack_sel=0;           slave_mux_sel=0;
slave_shift_piso=0;      slave_load_piso=0;         slave_shift_data=0;
slave_shift_addr=0;
end

else begin
slave_tri_en=1;          slave_ack_sel=0;           slave_mux_sel=0;
slave_shift_piso=0;      slave_load_piso=0;         slave_shift_data=0;
slave_shift_addr=0;
end
endmodule


module simple_slave(slave_reset,slave_scl_input,slave_sda_input,slave_data_out,slave_data);
input slave_reset;
input slave_scl_input;
input [7:0]slave_data;
output [7:0]slave_data_out;
inout slave_sda_input;
wire start_stop_detect;
wire slave_sda_in,slave_tri_en;
wire slave_shift_data,slave_load_data;
wire slave_rec_data_shift;
wire slave_sda_out;
wire slave_ack_sel,slave_mux_sel;
wire slave_ack_out,slave_serial_out_data,slave_rec_addr_shift,slave_rd_wr;
wire [6:0]slave_addr_out;

slave_start_stop_detector det1(.slave_reset(slave_reset),.slave_scl_in(slave_scl_input),.slave_sda_in(slave_sda_input),.start_stop_detect(start_stop_detect));
slave_tristate_logic tri12(.slave_sda_out(slave_sda_out),.slave_tri_en(slave_tri_en),.slave_sda_in(slave_sda_in),.slave_sda(slave_sda_input));
slave_sipo_addr sipo1(.slave_serial_in(slave_sda_in),.slave_scl_sixt(slave_scl_input),.slave_rec_addr_shift(slave_rec_addr_shift),.slave_addr_out(slave_addr_out),.slave_rd_wr(slave_rd_wr));
slave_sipo_data sipo2(.slave_serial_in(slave_sda_in),.slave_rec_data_shift(slave_rec_data_shift),.slave_scl_sixt(slave_scl_input),.slave_data_out(slave_data_out));
slave_piso_data piso1(.slave_data(slave_data),.slave_scl_sixt(slave_scl_input),.slave_load_data(slave_load_data),.slave_shift_data(slave_shift_data),.slave_serial_out_data(slave_serial_out_data));
slave_mux slavem1(.slave_mux_sel(slave_mux_sel),.slave_serial_out_data(slave_serial_out_data),.slave_mux_out(slave_sda_out),.slave_acknowledge(slave_ack_out));
slave_mux2x1 slavem2(.slave_ack_sel(slave_ack_sel),.slave_ack_out(slave_ack_out));
slave_fsm f23(.slave_tri_en(slave_tri_en),.slave_shift_addr(slave_rec_addr_shift),.slave_shift_piso(slave_shift_data),.slave_load_piso(slave_load_data),.slave_shift_data(slave_rec_data_shift), 
                                .slave_ack_sel(slave_ack_sel),.slave_mux_sel(slave_mux_sel),.slave_scl_sixt(slave_scl_input),.slave_ack(slave_sda_input),.slave_start_bit(start_stop_detect),.slave_addr_rcvd(slave_addr_out),.slave_rd_wr(slave_rd_wr),.slave_reset(slave_reset));
endmodule

module simple_slave_tb;
reg slave_fpga_clk,slave_reset;
reg slave_scl_input;
reg [7:0]slave_data;
wire [7:0]slave_data_out;
wire slave_sda_input;

simple_slave slave23(slave_fpga_clk,slave_reset,slave_scl_input,slave_sda_input,slave_data_out,slave_data);

always #4 slave_fpga_clk=~slave_fpga_clk;

initial begin
slave_reset=0;
#3 slave_reset=1;
end

initial begin
slave_data=8'b1111_0000;
end
endmodule


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module i2c_top(slave_reset,slave_data_out, slave_data, mast_start_bit, fpga_clk, mast_rd_wr,
    mast_address, mast_data, mast_rst, data_from_slave);
input mast_rst, slave_reset;
input [7:0]slave_data;
input  mast_start_bit, fpga_clk, mast_rd_wr;
input [7:0]mast_data;
input [6:0]mast_address;
output [7:0]slave_data_out;
output [7:0]data_from_slave;
wire scl;
wire sda;
wire scl_clk;

master_scl_generator scl45(.fpga_clock(fpga_clk),.scl_clock(scl_clk));
simple_master m45(.mast_start_bit(mast_start_bit),.fpga_clk(scl_clk),.mast_rd_wr(mast_rd_wr),.mast_address(mast_address),.mast_data(mast_data),.mast_rst(mast_rst),.scl(scl),.sda(sda),.data_from_slave(data_from_slave));
simple_slave s67(.slave_reset(slave_reset),.slave_scl_input(scl),.slave_sda_input(sda),.slave_data_out(slave_data_out),.slave_data(slave_data));
endmodule

module i2c_top_tb();
reg mast_rst, slave_reset;
reg [7:0]slave_data;
reg  mast_start_bit, mast_rd_wr;
reg  fpga_clk=0;
reg [7:0]mast_data;
reg [6:0]mast_address;
wire [7:0]slave_data_out;
wire [7:0]data_from_slave;

i2c_top i2c12(slave_reset,slave_data_out, slave_data, mast_start_bit, fpga_clk, mast_rd_wr,
    mast_address, mast_data, mast_rst, data_from_slave);

//always #4 fpga_clk=~fpga_clk;
always #10 fpga_clk=~fpga_clk;
//always #5 fpga_clk=~fpga_clk;

initial begin
mast_rst=0;slave_reset=0;
#4500 mast_rst=1;slave_reset=1;
end

initial begin
mast_data=8'b1111_0000; mast_rd_wr=0;mast_start_bit=1;mast_address=7'b1111010;
slave_data=8'b0000_1111;
#180000 mast_start_bit=0;
end
endmodule


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////master scl generator to generate 100khz scl
module master_scl_generator(fpga_clock, scl_clock);
input fpga_clock;
output scl_clock;
integer count=0;
                                
always @(posedge fpga_clock)
if(count<500)
count<=count+1;
else
count<=1;
                                
assign scl_clock= (count<251) ? 0: 1;
endmodule