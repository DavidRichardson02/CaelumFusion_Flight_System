`timescale 1ns/1ps
`default_nettype none

`include "telemetry_defs_vh.vh"

//==============================================================================
// tb_caelumfusion_science_page_ext_metadata
//------------------------------------------------------------------------------
// Pixel-level smoke test for the science pages' extension-metadata contract.
// The bench avoids text/legend assumptions and samples stable interior pixels
// from status cells, bars, and MAG metadata markers.
//==============================================================================
module tb_caelumfusion_science_page_ext_metadata;

    localparam [2:0] VIEW_SCIENCE_EXPLAIN   = 3'd4;
    localparam [2:0] VIEW_SCIENCE_WIND      = 3'd5;
    localparam [2:0] VIEW_SCIENCE_INTEGRITY = 3'd6;

    reg        sys_clk;
    reg        pix_clk;
    reg        sys_rst;
    reg        pix_rst;
    reg [2:0]  page_id_sys;

    reg        ext_valid;
    reg [7:0]  ext_status;
    reg [15:0] ext_present_flags;
    reg [15:0] ext_fault_flags;
    reg [15:0] ext_mag_delta_l1;
    reg [15:0] ext_mag_norm_primary;
    reg [15:0] ext_mag_norm_secondary;
    reg        ext_mag_sequence_aligned;
    reg        ext_mag_disagreement;
    reg [3:0]  ext_mag_sector_delta;
    reg [15:0] ext_mag_norm_delta_l1;
    reg [15:0] ext_mag_iron_residual;
    reg [7:0]  ext_mag_cal_state;
    reg [7:0]  ext_mag_source_flags;
    reg [15:0] ext_mag_bridge_checksum;
    reg [15:0] ext_rng_height_cm;
    reg [15:0] ext_air_dp_pa;
    reg [15:0] ext_air_speed_cms;
    reg [15:0] ext_env_temp_cdeg;
    reg [15:0] ext_env_rh_centi;
    reg [15:0] ext_sun_luma;
    reg [15:0] ext_flow_dx;
    reg [15:0] ext_flow_dy;
    reg [15:0] ext_log_seq;
    reg [15:0] ext_log_drop_count;
    reg [15:0] ext_max_age_ms;

    wire       vga_hsync_out;
    wire       vga_vsync_out;
    wire [11:0] vga_rgb_out;

    integer errors;

    initial begin
        sys_clk = 1'b0;
        pix_clk = 1'b0;
        forever #5 begin
            sys_clk = ~sys_clk;
            pix_clk = ~pix_clk;
        end
    end

    caelumfusion_science_page_vga dut (
        .sys_clk(sys_clk),
        .sys_rst(sys_rst),
        .page_id_sys(page_id_sys),
        .bmp_valid(1'b1),
        .bmp_status(`ST_OK),
        .bmp_age_ms(16'd12),
        .acc_valid(1'b1),
        .acc_status(`ST_OK),
        .acc_age_ms(16'd12),
        .mag_valid(1'b1),
        .mag_status(`ST_OK),
        .mag_payload(48'd0),
        .mag_age_ms(16'd12),
        .pwr_valid(1'b1),
        .pwr_status(`ST_OK),
        .pwr_payload(48'd0),
        .pwr_age_ms(16'd12),
        .ext_valid(ext_valid),
        .ext_status(ext_status),
        .ext_present_flags(ext_present_flags),
        .ext_fault_flags(ext_fault_flags),
        .ext_mag_delta_l1(ext_mag_delta_l1),
        .ext_mag_norm_primary(ext_mag_norm_primary),
        .ext_mag_norm_secondary(ext_mag_norm_secondary),
        .ext_mag_sequence_aligned(ext_mag_sequence_aligned),
        .ext_mag_disagreement(ext_mag_disagreement),
        .ext_mag_sector_delta(ext_mag_sector_delta),
        .ext_mag_norm_delta_l1(ext_mag_norm_delta_l1),
        .ext_mag_iron_residual(ext_mag_iron_residual),
        .ext_mag_cal_state(ext_mag_cal_state),
        .ext_mag_source_flags(ext_mag_source_flags),
        .ext_mag_bridge_checksum(ext_mag_bridge_checksum),
        .ext_rng_height_cm(ext_rng_height_cm),
        .ext_air_dp_pa(ext_air_dp_pa),
        .ext_air_speed_cms(ext_air_speed_cms),
        .ext_env_temp_cdeg(ext_env_temp_cdeg),
        .ext_env_rh_centi(ext_env_rh_centi),
        .ext_sun_luma(ext_sun_luma),
        .ext_flow_dx(ext_flow_dx),
        .ext_flow_dy(ext_flow_dy),
        .ext_log_seq(ext_log_seq),
        .ext_log_drop_count(ext_log_drop_count),
        .ext_max_age_ms(ext_max_age_ms),
        .der_valid(1'b1),
        .der_status(`ST_OK),
        .der_alt_fresh(1'b1),
        .der_vspd_fresh(1'b1),
        .der_bmp_age_ms(16'd12),
        .der_acc_age_ms(16'd12),
        .der_mag_age_ms(16'd12),
        .der_altitude_cm(32'd0),
        .der_vertical_speed_cms(32'd0),
        .nav_valid(1'b1),
        .nav_status(`ST_OK),
        .nav_downrange_m(16'd64),
        .nav_crossrange_m(16'd48),
        .nav_age_ms(16'd12),
        .wind_valid(1'b1),
        .wind_status(`ST_OK),
        .wind_x_cms(16'd512),
        .wind_y_cms(16'd384),
        .wind_age_ms(16'd12),
        .auth_phase_code_sys(4'd9),
        .auth_phase_valid_sys(1'b1),
        .safety_runtime_ok_sys(1'b1),
        .safety_allows_actuation_sys(1'b1),
        .policy_runtime_enable_sys(1'b1),
        .software_armed_sys(1'b1),
        .pix_clk(pix_clk),
        .pix_rst(pix_rst),
        .vga_hsync_in(1'b1),
        .vga_vsync_in(1'b1),
        .vga_rgb_in(12'h001),
        .vga_hsync_out(vga_hsync_out),
        .vga_vsync_out(vga_vsync_out),
        .vga_rgb_out(vga_rgb_out)
    );

    task fail;
        input [8*96-1:0] msg;
        begin
            errors = errors + 1;
            $display("FAIL: %0s at %0t", msg, $time);
        end
    endtask

    task check_rgb;
        input [8*96-1:0] msg;
        input [11:0] actual;
        input [11:0] expected;
        begin
            if (actual !== expected) begin
                $display("FAIL: %0s actual=%03h expected=%03h at %0t",
                         msg, actual, expected, $time);
                errors = errors + 1;
            end
        end
    endtask

    task wait_snapshot;
        begin
            repeat (70000) @(posedge sys_clk);
        end
    endtask

    task wait_pixel;
        input [10:0] px;
        input [10:0] py;
        begin
            while (!((dut.h_count == px) &&
                     (dut.v_count == py) &&
                     (dut.active_pix == 1'b1))) begin
                @(posedge pix_clk);
            end
            #1;
        end
    endtask

    task configure_nominal_extension;
        begin
            ext_valid = 1'b1;
            ext_status = `ST_OK;
            ext_present_flags = 16'd0;
            ext_present_flags[`EXT_PRESENT_RANGE_BIT]    = 1'b1;
            ext_present_flags[`EXT_PRESENT_AIR_BIT]      = 1'b1;
            ext_present_flags[`EXT_PRESENT_ENV_BIT]      = 1'b1;
            ext_present_flags[`EXT_PRESENT_SUN_BIT]      = 1'b1;
            ext_present_flags[`EXT_PRESENT_FLOW_BIT]     = 1'b1;
            ext_present_flags[`EXT_PRESENT_MAG1_BIT]     = 1'b1;
            ext_present_flags[`EXT_PRESENT_BLACKBOX_BIT] = 1'b1;
            ext_fault_flags = 16'd0;
            ext_mag_delta_l1 = 16'd64;
            ext_mag_norm_primary = 16'd2400;
            ext_mag_norm_secondary = 16'd2450;
            ext_mag_sequence_aligned = 1'b1;
            ext_mag_disagreement = 1'b0;
            ext_mag_sector_delta = 4'd3;
            ext_mag_norm_delta_l1 = 16'd50;
            ext_mag_iron_residual = 16'd512;
            ext_mag_cal_state = 8'h01;
            ext_mag_source_flags = 8'd0;
            ext_mag_source_flags[`EXT_SRC_REAL_BIT] = 1'b1;
            ext_mag_bridge_checksum = 16'hBEEF;
            ext_rng_height_cm = 16'd3200;
            ext_air_dp_pa = 16'd512;
            ext_air_speed_cms = 16'd1600;
            ext_env_temp_cdeg = 16'd2500;
            ext_env_rh_centi = 16'd5000;
            ext_sun_luma = 16'd2048;
            ext_flow_dx = 16'd1600;
            ext_flow_dy = 16'hF380; // -3200 two's-complement.
            ext_log_seq = 16'h0134;
            ext_log_drop_count = 16'd0;
            ext_max_age_ms = 16'd44;
        end
    endtask

    initial begin
        errors = 0;
        sys_rst = 1'b1;
        pix_rst = 1'b1;
        page_id_sys = VIEW_SCIENCE_EXPLAIN;
        configure_nominal_extension();

        repeat (8) @(posedge sys_clk);
        sys_rst = 1'b0;
        pix_rst = 1'b0;

        wait_snapshot();
        wait_pixel(11'd25, 11'd50);
        check_rgb("explain page range presence cell uses extension status",
                  vga_rgb_out, 12'h2d5);
        wait_pixel(11'd70, 11'd300);
        check_rgb("explain page range-height bar uses extension range metadata",
                  vga_rgb_out, 12'h2d5);

        page_id_sys = VIEW_SCIENCE_WIND;
        wait_snapshot();
        wait_pixel(11'd40, 11'd120);
        check_rgb("wind page flow status cell uses extension flow metadata",
                  vga_rgb_out, 12'h2d5);
        wait_pixel(11'd30, 11'd436);
        check_rgb("wind page bottom flow bar uses extension flow metadata",
                  vga_rgb_out, 12'h2d5);

        page_id_sys = VIEW_SCIENCE_INTEGRITY;
        wait_snapshot();
        wait_pixel(11'd40, 11'd105);
        check_rgb("integrity page MAG0 norm bar uses extension MAG metadata",
                  vga_rgb_out, 12'h2d5);
        wait_pixel(11'd86, 11'd190);
        check_rgb("integrity page MAG sector marker uses extension sector metadata",
                  vga_rgb_out, 12'hfff);
        wait_pixel(11'd40, 11'd255);
        check_rgb("integrity page MAG residual bar uses extension calibration metadata",
                  vga_rgb_out, 12'h2d5);

        if (vga_hsync_out !== 1'b0 && vga_hsync_out !== 1'b1)
            fail("hsync output is unknown");
        if (vga_vsync_out !== 1'b0 && vga_vsync_out !== 1'b1)
            fail("vsync output is unknown");

        if (errors == 0) begin
            $display("PASS: tb_caelumfusion_science_page_ext_metadata");
        end else begin
            $display("FAIL: tb_caelumfusion_science_page_ext_metadata errors=%0d", errors);
        end
        $finish;
    end

endmodule

`default_nettype wire
