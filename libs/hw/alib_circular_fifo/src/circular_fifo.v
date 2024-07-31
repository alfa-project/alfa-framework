module alib_circular_fifo #(
    parameter DEPTH = 16,
    parameter WIDTH = 8
) (
    input wire clk,
    input wire rst,
    input wire [WIDTH-1:0] data_in,
    input wire wr_en,
    input wire rd_en,
    output reg [WIDTH-1:0] data_out,
    output wire full,
    output wire empty
);

  reg [WIDTH-1:0] fifo [0:DEPTH-1];
  reg [$clog2(DEPTH)-1:0] head;
  reg [$clog2(DEPTH)-1:0] tail;
  reg [$clog2(DEPTH)-1:0] count;

  assign full  = (count == DEPTH);
  assign empty = (count == 0);
  integer i;

  always @(posedge clk) begin
    if (rst) begin
      if (wr_en && !full) begin
        fifo[head] <= data_in;
        head <= (head + 1) % DEPTH;
        count <= count + 1;
      end
      if (rd_en && !empty) begin
        data_out <= fifo[tail];
        tail <= (tail + 1) % DEPTH;
        count <= count - 1;
      end
    end else begin
      head <= 0;
      tail <= 0;
      count <= 0;
      data_out <= 0;
      for (i = 0; i < DEPTH; i = i + 1) begin
        fifo[i] <= 0;
      end
    end
  end

endmodule

module points_circular_fifo #(
    parameter DEPTH = 16
) (
    input wire clk,
    input wire rst,
    input wire [15:0] point_x_in,
    input wire [15:0] point_y_in,
    input wire [15:0] point_z_in,
    input wire wr_en,
    input wire rd_en,
    output reg [15:0] point_x_out,
    output reg [15:0] point_y_out,
    output reg [15:0] point_z_out,
    output wire full,
    output wire empty
);

  reg [15:0] fifo_x[0:DEPTH-1];
  reg [15:0] fifo_y[0:DEPTH-1];
  reg [15:0] fifo_z[0:DEPTH-1];
  reg [ 7:0] head;
  reg [ 7:0] tail;
  reg [ 7:0] count;

  assign full  = (count == DEPTH);
  assign empty = (count == 0);
  integer i;

  always @(posedge clk) begin
    if (rst) begin
      if (wr_en && !full) begin
        fifo_x[head] <= point_x_in;
        fifo_y[head] <= point_y_in;
        fifo_z[head] <= point_z_in;
        head <= (head + 1) % DEPTH;
        count <= count + 1;
      end
      if (rd_en && !empty) begin
        point_x_out <= fifo_x[tail];
        point_y_out <= fifo_y[tail];
        point_z_out <= fifo_z[tail];
        tail <= (tail + 1) % DEPTH;
        count <= count - 1;
      end
    end else begin
      head <= 0;
      tail <= 0;
      count <= 0;
      point_z_out <= 0;
      point_y_out <= 0;
      point_x_out <= 0;
      for (i = 0; i < DEPTH; i = i + 1) begin
        fifo_x[i] <= 0;
        fifo_y[i] <= 0;
        fifo_z[i] <= 0;
      end
    end
  end

endmodule

