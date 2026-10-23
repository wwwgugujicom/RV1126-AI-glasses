// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * RV1126B F (AL schematic V1.0) - Quad camera DTSI
 * 4x independent DPHY, one camera per pipeline.
 *
 * Actual (enabled) population on this build: 4x SC132GS
 *   CAM0 : i2c3, m-idx0, reset GPIO6_PA1 -> dphy1 -> mipi0_csi2 -> rkcif_mipi_lvds  -> isp_vir0
 *   CAM1 : i2c3, m-idx1, reset GPIO6_PA2 -> dphy2 -> mipi1_csi2 -> rkcif_mipi_lvds1 -> isp_vir1
 *   CAM2 : i2c4, m-idx2, reset GPIO6_PA3 -> dphy4 -> mipi2_csi2 -> rkcif_mipi_lvds2 -> isp_vir2
 *   CAM3 : i2c4, m-idx3, reset GPIO6_PA4 -> dphy5 -> mipi3_csi2 -> rkcif_mipi_lvds3 -> isp_vir3
 *
 * Per-slot alternative (SC533HGS) pins are present as *disabled* placeholders.
 * NOTE: node labels sc533hgs_1 / sc533hgs_4 keep the original names to avoid
 *       breaking board-level references.
 */

/* ====================== System regulators ====================== */
/ {

    vcc3v0_sys: vcc-3v0-sys {
        compatible     = "regulator-fixed";
        regulator-name = "vcc3v0_sys";
        regulator-min-microvolt = <3000000>;
        regulator-max-microvolt = <3000000>;
        regulator-always-on;
        regulator-boot-on;
        vin-supply = <&vcc3v3_sys>;
        status     = "okay";
    };

    vcc2v8_sys: vcc-2v8-sys {
        compatible      = "regulator-fixed";
        regulator-name  = "vcc2v8_sys";
        regulator-min-microvolt = <2800000>;
        regulator-max-microvolt = <2800000>;
        regulator-always-on;
        regulator-boot-on;
        vin-supply = <&vcc3v3_sys>;
        status     = "okay";
    };
    vcc1v8_sys: vcc-1v8-sys {
        compatible     = "regulator-fixed";
        regulator-name = "vcc1v8_sys";
        regulator-min-microvolt = <1800000>;
        regulator-max-microvolt = <1800000>;
        regulator-always-on;
        regulator-boot-on;
        vin-supply = <&vcc3v3_sys>;
        status     = "okay";
    };
    vcc1v2_sys: vcc-1v2-sys {
        compatible     = "regulator-fixed";
        regulator-name = "vcc1v2_sys";
        regulator-min-microvolt = <1200000>;
        regulator-max-microvolt = <1200000>;
        regulator-always-on;
        regulator-boot-on;
        vin-supply = <&vcc3v3_sys>;
        status     = "okay";
    };
};

/* Disable unused DPHYs */
&csi2_dphy0 { 
    status = "disabled";
};

&csi2_dphy3 { 
    status = "disabled"; 
};

/* ====================== DPHY1 - CAM0 ====================== */
&csi2_dphy1 {
    status = "okay";
    ports {
        #address-cells = <1>;
        #size-cells    = <0>;
        port@0 {
            reg = <0>;
            #address-cells = <1>;
            #size-cells    = <0>;
            csi2_dphy1_input0: endpoint@0 {
                reg = <0>;
                /* active sensor on this slot */
                remote-endpoint = <&sc132gs_1_out>;
                data-lanes      = <1 2>;
            };

            csi2_dphy1_input1: endpoint@1 {
                reg = <1>;
                /* active sensor on this slot */
                remote-endpoint = <&sc533hgs_1_out>;
                data-lanes      = <1 2>;
            };

            csi2_dphy1_input2: endpoint@2 {
                reg = <2>;
                /* active sensor on this slot */
                remote-endpoint = <&sc235hgs_2_out>;
                data-lanes      = <1 2>;
            };
        };
        port@1 {
            reg = <1>;
            #address-cells = <1>;
            #size-cells    = <0>;
            csidphy1_out: endpoint@0 {
                reg = <0>;
                remote-endpoint = <&mipi0_csi2_input>;
            };
        };
    };
};

/* ====================== DPHY2 - CAM1 ====================== */
&csi2_dphy2 {
    status = "okay";
    ports {
        #address-cells = <1>;
        #size-cells    = <0>;
        port@0 {
            reg = <0>;
            #address-cells = <1>;
            #size-cells    = <0>;
            csi2_dphy2_input0: endpoint@0 {
                reg = <0>;
                remote-endpoint = <&sc132gs_2_out>;
                data-lanes      = <1 2>;
            };

            csi2_dphy2_input1: endpoint@1 {
                reg = <1>;
                remote-endpoint = <&sc132gs_5_out>;
                data-lanes      = <1 2>;
            };
        };

        port@1 {
            reg = <1>;
            #address-cells = <1>;
            #size-cells    = <0>;
            csidphy2_out: endpoint@0 {
                reg = <0>;
                remote-endpoint = <&mipi1_csi2_input>;
            };
        };
    };
};

/* ====================== DPHY4 - CAM2 ====================== */
&csi2_dphy4 {
    status = "okay";
    ports {
        #address-cells = <1>;
        #size-cells    = <0>;
        port@0 {
            reg = <0>;
            #address-cells = <1>;
            #size-cells    = <0>;
            csi2_dphy4_input0: endpoint@0 {
                reg = <0>;
                remote-endpoint = <&sc132gs_4_out>;
                data-lanes      = <1 2>;
            };
            csi2_dphy4_input1: endpoint@1 {
                reg = <1>;
                remote-endpoint = <&sc533hgs_4_out>;
                data-lanes      = <1 2>;
            };

            csi2_dphy4_input2: endpoint@2 {
                reg = <2>;
                remote-endpoint = <&sc235hgs_1_out>;
                data-lanes      = <1 2>;
            };
        };
        port@1 {
            reg = <1>;
            #address-cells = <1>;
            #size-cells    = <0>;
            csidphy4_out: endpoint@0 {
                reg = <0>;
                remote-endpoint = <&mipi2_csi2_input>;
            };
        };
    };
};

/* ====================== DPHY5 - CAM3 ====================== */
&csi2_dphy5 {
    status = "okay";
    ports {
        #address-cells = <1>;
        #size-cells    = <0>;
        port@0 {
            reg = <0>;
            #address-cells = <1>;
            #size-cells    = <0>;
            csi2_dphy5_input0: endpoint@0 {
                reg = <0>;
                remote-endpoint = <&sc132gs_3_out>;
                data-lanes      = <1 2>;
            };
            csi2_dphy5_input1: endpoint@1 {
                reg = <1>;
                remote-endpoint = <&sc132gs_6_out>;
                data-lanes      = <1 2>;
            };
        };
        port@1 {
            reg = <1>;
            #address-cells = <1>;
            #size-cells    = <0>;
            csidphy5_out: endpoint@0 {
                reg = <0>;
                remote-endpoint = <&mipi3_csi2_input>;
            };
        };
    };
};

/* ====================== I2C3 - CAM0 (m-idx0) + CAM1 (m-idx1) ====================== */
&i2c3 {
/***********************************************************sc132gs+sc533hgs*******************************************************/
    sc533hgs_1: sc533hgs-1@30 {
        compatible = "smartsens,sc533hgs";
        status = "disabled";
        reg = <0x30>;
        clocks = <&cru CLK_MIPI0_OUT2IO>;
        clock-names  = "xvclk";
        avdd-supply  = <&vcc2v8_sys>;
        dovdd-supply = <&vcc1v8_sys>;
        dvdd-supply  = <&vcc1v2_sys>;
        reset-gpios  = <&gpio6 RK_PA1 GPIO_ACTIVE_HIGH>;
        pinctrl-names = "default";
        pinctrl-0     = <&cam_clk0_pins>;
        rockchip,camera-module-index     = <0>;
        rockchip,camera-module-facing    = "back";
        rockchip,camera-module-name      = "default";
        rockchip,camera-module-lens-name = "default";
        port {
            sc533hgs_1_out: endpoint {
                remote-endpoint = <&csi2_dphy1_input1>;
                data-lanes      = <1 2>;
            };
        };
    };

    sc132gs_5: sc132gs-5@31 {        
        compatible = "smartsens,sc132gs";
        status = "disabled";
        reg = <0x31>;
        clocks = <&cru CLK_MIPI1_OUT2IO>;
        clock-names  = "xvclk";
        avdd-supply  = <&vcc2v8_sys>;
        dovdd-supply = <&vcc1v8_sys>;
        dvdd-supply  = <&vcc1v2_sys>;
        reset-gpios  = <&gpio6 RK_PA2 GPIO_ACTIVE_LOW>;
        pinctrl-names = "default";
        pinctrl-0     = <&cam_clk1_pins>;
        rockchip,camera-module-index     = <1>;
        rockchip,camera-module-facing    = "back";
        rockchip,camera-module-name      = "default";
        rockchip,camera-module-lens-name = "default";
        port {
            sc132gs_5_out: endpoint {
                remote-endpoint = <&csi2_dphy2_input1>;
                data-lanes      = <1 2>;
            };
        };
    };
/***********************************************************************************************************************************/


/***********************************************************sc132+sc132*************************************************************/
    sc132gs_1: sc132gs-1@32 {
        compatible = "smartsens,sc132gs";
        status = "disabled";
        reg = <0x32>;
        clocks = <&cru CLK_MIPI0_OUT2IO>;
        clock-names  = "xvclk";
        avdd-supply  = <&vcc2v8_sys>;
        dovdd-supply = <&vcc1v8_sys>;
        dvdd-supply  = <&vcc1v2_sys>;
        reset-gpios  = <&gpio6 RK_PA1 GPIO_ACTIVE_LOW>;
        pinctrl-names = "default";
        pinctrl-0     = <&cam_clk0_pins>;
        rockchip,camera-module-index     = <0>;
        rockchip,camera-module-facing    = "back";
        rockchip,camera-module-name      = "default";
        rockchip,camera-module-lens-name = "default";
        port {
            sc132gs_1_out: endpoint {
                remote-endpoint = <&csi2_dphy1_input0>;
                data-lanes      = <1 2>;
            };
        };
    };

    sc132gs_2: sc132gs-2@33 {
        compatible = "smartsens,sc132gs";
        status = "disabled";
        reg = <0x33>;
        clocks = <&cru CLK_MIPI1_OUT2IO>;
        clock-names  = "xvclk";
        avdd-supply  = <&vcc2v8_sys>;
        dovdd-supply = <&vcc1v8_sys>;
        dvdd-supply  = <&vcc1v2_sys>;
        reset-gpios  = <&gpio6 RK_PA2 GPIO_ACTIVE_LOW>;
        pinctrl-names = "default";
        pinctrl-0     = <&cam_clk1_pins>;
        rockchip,camera-module-index     = <1>;
        rockchip,camera-module-facing    = "back";
        rockchip,camera-module-name      = "default";
        rockchip,camera-module-lens-name = "default";
        port {
            sc132gs_2_out: endpoint {
                remote-endpoint = <&csi2_dphy2_input0>;
                data-lanes      = <1 2>;
            };
        };
    };
/***********************************************************************************************************************************/

/********************************************sc235hgs*******************************************************************************/
    sc235hgs_2:sc235hgs-2@32 {
        compatible = "smartsens,sc235hgs";
        status = "okay";
        reg = <0x32>;
        clocks = <&cru CLK_MIPI0_OUT2IO>;
        clock-names  = "xvclk";
        avdd-supply  = <&vcc3v0_sys>;
        dovdd-supply = <&vcc1v8_sys>;
        dvdd-supply  = <&vcc1v2_sys>;
        reset-gpios  = <&gpio6 RK_PA1 GPIO_ACTIVE_HIGH>;
        pinctrl-names = "default";
        pinctrl-0     = <&cam_clk0_pins>;
        rockchip,camera-module-index     = <0>;
        rockchip,camera-module-facing    = "back";
        rockchip,camera-module-name      = "default";
        rockchip,camera-module-lens-name = "default";
        port {
            sc235hgs_2_out: endpoint {
                remote-endpoint = <&csi2_dphy1_input2>;
                data-lanes      = <1 2>;
            };
        };
    };
/*************************************************** IMU: ICM-42670 / ICM-42688-P **************************************************/
    icm42670: imu@68 {
        status = "okay";
        reg = <0x68>;
        /* driver auto-detects by WHO_AM_I: 0x67=ICM-42670, 0x47=ICM-42688-P */
        compatible   = "invensense,icm42670", "invensense,icm42688";
        vdd-supply   = <&vcc3v3_sys>;
        vddio-supply = <&vcc1v8_sys>;
    };
};

&i2c4 {
/***********************************************************sc132gs+sc533hgs********************************************************/
    sc132gs_6: sc132gs-6@33 {            
        compatible = "smartsens,sc132gs";
        status = "disabled";
        reg = <0x33>;
        clocks = <&cru CLK_MIPI3_OUT2IO>;
        clock-names  = "xvclk";
        avdd-supply  = <&vcc2v8_sys>;
        dovdd-supply = <&vcc1v8_sys>;
        dvdd-supply  = <&vcc1v2_sys>;
        reset-gpios  = <&gpio6 RK_PA4 GPIO_ACTIVE_LOW>;
        pinctrl-names = "default";
        pinctrl-0     = <&cam_clk3_pins>;
        rockchip,camera-module-index     = <3>;
        rockchip,camera-module-facing    = "back";
        rockchip,camera-module-name      = "default";
        rockchip,camera-module-lens-name = "default";
        port {
            sc132gs_6_out: endpoint {
                remote-endpoint = <&csi2_dphy5_input1>;
                data-lanes      = <1 2>;
            };
        };
    };

    sc533gs_4: sc533gs-4@32 {
        compatible = "smartsens,sc533hgs";
        status = "disabled";
        reg = <0x32>;
        clocks = <&cru CLK_MIPI2_OUT2IO>;
        clock-names  = "xvclk";
        avdd-supply  = <&vcc2v8_sys>;
        dovdd-supply = <&vcc1v8_sys>;
        dvdd-supply  = <&vcc1v2_sys>;
        reset-gpios  = <&gpio6 RK_PA3 GPIO_ACTIVE_HIGH>;
        pinctrl-names = "default";
        pinctrl-0     = <&cam_clk2_pins>;
        rockchip,camera-module-index     = <2>;
        rockchip,camera-module-facing    = "back";
        rockchip,camera-module-name      = "default";
        rockchip,camera-module-lens-name = "default";
        port {
            sc533hgs_4_out: endpoint {
                remote-endpoint = <&csi2_dphy4_input1>;
                data-lanes      = <1 2>;
            };
        };
    };
/***********************************************************************************************************************************/

/***********************************************************sc132+sc132*************************************************************/
    sc132gs_3: sc132gs-3@31 {
        compatible = "smartsens,sc132gs";
        status = "disabled";
        reg = <0x31>;
        clocks = <&cru CLK_MIPI3_OUT2IO>;
        clock-names  = "xvclk";
        avdd-supply  = <&vcc2v8_sys>;
        dovdd-supply = <&vcc1v8_sys>;
        dvdd-supply  = <&vcc1v2_sys>;
        reset-gpios  = <&gpio6 RK_PA4 GPIO_ACTIVE_LOW>;
        pinctrl-names = "default";
        pinctrl-0     = <&cam_clk3_pins>;
        rockchip,camera-module-index     = <3>;
        rockchip,camera-module-facing    = "back";
        rockchip,camera-module-name      = "default";
        rockchip,camera-module-lens-name = "default";
        port {
            sc132gs_3_out: endpoint {
                remote-endpoint = <&csi2_dphy5_input0>;
                data-lanes      = <1 2>;
            };
        };
    };

    sc132gs_4: sc132gs-4@30 {
        compatible = "smartsens,sc132gs";
        status = "disabled";
        reg = <0x30>;
        clocks = <&cru CLK_MIPI2_OUT2IO>;
        clock-names  = "xvclk";
        avdd-supply  = <&vcc2v8_sys>;
        dovdd-supply = <&vcc1v8_sys>;
        dvdd-supply  = <&vcc1v2_sys>;
        reset-gpios  = <&gpio6 RK_PA3 GPIO_ACTIVE_LOW>;
        pinctrl-names = "default";
        pinctrl-0     = <&cam_clk2_pins>;
        rockchip,camera-module-index     = <2>;
        rockchip,camera-module-facing    = "back";
        rockchip,camera-module-name      = "default";
        rockchip,camera-module-lens-name = "default";
        port {
            /* FIX: was wrongly labeled sc533hgs_4_out (duplicate label) */
            sc132gs_4_out: endpoint {
                remote-endpoint = <&csi2_dphy4_input0>;
                data-lanes      = <1 2>;
            };
        };
    };
/***********************************************************************************************************************************/

/*****************************************************************sc235hgs**********************************************************/
    sc235hgs_1:sc235hgs-1@30 {
        compatible = "smartsens,sc235hgs";
        status = "okay";
        reg = <0x30>;
        clocks = <&cru CLK_MIPI2_OUT2IO>;
        clock-names  = "xvclk";
        avdd-supply  = <&vcc3v0_sys>;
        dovdd-supply = <&vcc1v8_sys>;
        dvdd-supply  = <&vcc1v2_sys>;
        reset-gpios  = <&gpio6 RK_PA3 GPIO_ACTIVE_HIGH>;
        pinctrl-names = "default";
        pinctrl-0     = <&cam_clk2_pins>;
        rockchip,camera-module-index     = <2>;
        rockchip,camera-module-facing    = "back";
        rockchip,camera-module-name      = "default";
        rockchip,camera-module-lens-name = "default";
        port {
            sc235hgs_1_out: endpoint {
                remote-endpoint = <&csi2_dphy4_input2>;
                data-lanes      = <1 2>;
            };
        };
    };
};

/* ====================== MIPI CSI2 Controllers ====================== */
&mipi0_csi2 {
    status = "okay";
    ports {
        #address-cells = <1>;
        #size-cells = <0>;
        port@0 {
            reg = <0>;
            #address-cells = <1>;
            #size-cells = <0>;
            mipi0_csi2_input: endpoint@0 {
                reg = <0>;
                remote-endpoint = <&csidphy1_out>;
            };
        };
        port@1 {
            reg = <1>;
            #address-cells = <1>;
            #size-cells = <0>;
            mipi0_csi2_output: endpoint@0 {
                reg = <0>;
                remote-endpoint = <&cif_mipi_in0>;
            };
        };
    };
};

&mipi1_csi2 {
    status = "okay";
    ports {
        #address-cells = <1>;
        #size-cells = <0>;
        port@0 {
            reg = <0>;
            #address-cells = <1>;
            #size-cells = <0>;
            mipi1_csi2_input: endpoint@0 {
                reg = <0>;
                remote-endpoint = <&csidphy2_out>;
            };
        };
        port@1 {
            reg = <1>;
            #address-cells = <1>;
            #size-cells = <0>;
            mipi1_csi2_output: endpoint@0 {
                reg = <0>;
                remote-endpoint = <&cif_mipi_in1>;
            };
        };
    };
};

&mipi2_csi2 {
    status = "okay";
    ports {
        #address-cells = <1>;
        #size-cells = <0>;
        port@0 {
            reg = <0>;
            #address-cells = <1>;
            #size-cells = <0>;
            mipi2_csi2_input: endpoint@0 {
                reg = <0>;
                remote-endpoint = <&csidphy4_out>;
            };
        };
        port@1 {
            reg = <1>;
            #address-cells = <1>;
            #size-cells = <0>;
            mipi2_csi2_output: endpoint@0 {
                reg = <0>;
                remote-endpoint = <&cif_mipi_in2>;
            };
        };
    };
};

&mipi3_csi2 {
    status = "okay";
    ports {
        #address-cells = <1>;
        #size-cells = <0>;
        port@0 {
            reg = <0>;
            #address-cells = <1>;
            #size-cells = <0>;
            mipi3_csi2_input: endpoint@0 {
                reg = <0>;
                remote-endpoint = <&csidphy5_out>;
            };
        };
        port@1 {
            reg = <1>;
            #address-cells = <1>;
            #size-cells = <0>;
            mipi3_csi2_output: endpoint@0 {
                reg = <0>;
                remote-endpoint = <&cif_mipi_in3>;
            };
        };
    };
};

/* ====================== RKCIF ====================== */
&rkcif { status = "okay"; };
&rkcif_mmu { status = "okay"; };

&rkcif_mipi_lvds {
    status = "okay";
    port { 
        cif_mipi_in0: endpoint { 
            remote-endpoint = <&mipi0_csi2_output>; 
        }; 
    };
};
&rkcif_mipi_lvds_sditf {
    status = "okay";
    port { 
        mipi_lvds_sditf: endpoint { 
            remote-endpoint = <&isp_vir0>; 
        }; 
    };
};

&rkcif_mipi_lvds1 {
    status = "okay";
    port { 
        cif_mipi_in1: endpoint { 
            remote-endpoint = <&mipi1_csi2_output>; 
        }; 
    };
};

&rkcif_mipi_lvds1_sditf {
    status = "okay";
    port { 
        mipi_lvds1_sditf: endpoint { 
            remote-endpoint = <&isp_vir1>; 
        }; 
    };
};

/* FIX: was cross-wired to mipi3_csi2_output */
&rkcif_mipi_lvds2 {
    status = "okay";
    port { 
        cif_mipi_in2: endpoint { 
            remote-endpoint = <&mipi2_csi2_output>; 
        }; 
    };
};
&rkcif_mipi_lvds2_sditf {
    status = "okay";
    port { 
        mipi_lvds2_sditf: endpoint { 
            remote-endpoint = <&isp_vir2>; 
        }; 
    };
};

/* FIX: was cross-wired to mipi2_csi2_output */
&rkcif_mipi_lvds3 {
    status = "okay";
    port { 
        cif_mipi_in3: endpoint { 
            remote-endpoint = <&mipi3_csi2_output>; 
        }; 
    };
};
&rkcif_mipi_lvds3_sditf {
    status = "okay";
    port { 
        mipi_lvds3_sditf: endpoint { 
            remote-endpoint = <&isp_vir3>; 
        }; 
    };
};

/* ====================== RKISP ====================== */
&rkisp { status = "okay"; };
&rkisp_mmu { status = "okay"; };

&rkisp_vir0 {
    status = "okay";
    port { 
        isp_vir0: endpoint@0 { 
            reg = <0>; 
            remote-endpoint = <&mipi_lvds_sditf>; 
        }; 
    };
};
&rkisp_vir0_sditf { status = "okay"; };

&rkisp_vir1 {
    status = "okay";
    port { 
        isp_vir1: endpoint@0 { 
            reg = <0>; 
            remote-endpoint = <&mipi_lvds1_sditf>; 
        }; 
    };
};

&rkisp_vir1_sditf { status = "okay"; };

&rkisp_vir2 {
    status = "okay";
    port { 
        isp_vir2: endpoint@0 { 
            reg = <0>; 
            remote-endpoint = <&mipi_lvds2_sditf>; 
        }; 
    };
};
&rkisp_vir2_sditf { status = "okay"; };

&rkisp_vir3 {
    status = "okay";
    port { 
        isp_vir3: endpoint@0 { 
            reg = <0>; 
            remote-endpoint = <&mipi_lvds3_sditf>; 
        }; 
    };
};


&rkisp_vir3_sditf { status = "okay"; };

/* ====================== RKVPSS ====================== */
&rkvpss { 
    status = "okay"; 
    dvbm = <&rkdvbm>; 
};
&rkvpss_mmu { status = "okay"; };
&rkvpss_vir0 { status = "okay"; };
&rkvpss_vir1 { status = "okay"; };
&rkvpss_vir2 { status = "okay"; };
&rkvpss_vir3 { status = "okay"; };

/* ====================== PWM0 CH5 - Frame Sync Trigger ====================== */
/* GPIO4_PA3 as PWM0 output for FSYNC trigger signal
 * 33333333ns = ~30fps frame sync
 * pinctrl: pwm0m1_ch5_pins = GPIO4_PA3 mux4 (pwm0 function)
 */
&pwm0_8ch_5 {
    status = "okay";
    pinctrl-names = "active";
    pinctrl-0 = <&pwm0m1_ch5_pins>;
};