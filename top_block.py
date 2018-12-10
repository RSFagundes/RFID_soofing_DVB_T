#!/usr/bin/env python
##################################################
# Gnuradio Python Flow Graph
# Title: Top Block
# Generated: Fri Mar 14 16:22:06 2014
##################################################

from gnuradio import blocks
from gnuradio import digital
from gnuradio import eng_notation
from gnuradio import filter
from gnuradio import gr
from gnuradio import wxgui
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from gnuradio.wxgui import scopesink2
from grc_gnuradio import wxgui as grc_wxgui
from optparse import OptionParser
import osmosdr
import wx

class top_block(grc_wxgui.top_block_gui):

    def __init__(self):
        grc_wxgui.top_block_gui.__init__(self, title="Top Block")
        _icon_path = "/usr/share/icons/hicolor/32x32/apps/gnuradio-grc.png"
        self.SetIcon(wx.Icon(_icon_path, wx.BITMAP_TYPE_ANY))

        ##################################################
        # Variables
        ##################################################
        self.rfid = rfid = 13.56e6
        self.subport = subport = rfid/16
        self.sym_rate = sym_rate = subport/8
        self.samp_rate = samp_rate = rfid/8
        self.dec = dec = 8
        self.upconv = upconv = 125e6
        self.samp_per_sym = samp_per_sym = (samp_rate/dec)/(sym_rate)

        ##################################################
        # Blocks
        ##################################################
        self.wxgui_scopesink2_1 = scopesink2.scope_sink_f(
        	self.GetWin(),
        	title="Scope Plot",
        	sample_rate=samp_rate/dec,
        	v_scale=0,
        	v_offset=0,
        	t_scale=0,
        	ac_couple=False,
        	xy_mode=False,
        	num_inputs=2,
        	trig_mode=wxgui.TRIG_MODE_AUTO,
        	y_axis_label="Counts",
        )
        self.Add(self.wxgui_scopesink2_1.win)
        self.rtlsdr_source_0 = osmosdr.source( args="numchan=" + str(1) + " " + "" )
        self.rtlsdr_source_0.set_sample_rate(samp_rate)
        self.rtlsdr_source_0.set_center_freq(rfid+upconv+subport, 0)
        self.rtlsdr_source_0.set_freq_corr(0, 0)
        self.rtlsdr_source_0.set_dc_offset_mode(0, 0)
        self.rtlsdr_source_0.set_iq_balance_mode(2, 0)
        self.rtlsdr_source_0.set_gain_mode(0, 0)
        self.rtlsdr_source_0.set_gain(5, 0)
        self.rtlsdr_source_0.set_if_gain(5, 0)
        self.rtlsdr_source_0.set_bb_gain(5, 0)
        self.rtlsdr_source_0.set_antenna("", 0)
        self.rtlsdr_source_0.set_bandwidth(0, 0)
          
        self.rational_resampler_xxx_1 = filter.rational_resampler_ccc(
                interpolation=1,
                decimation=int(samp_per_sym),
                taps=None,
                fractional_bw=None,
        )
        self.rational_resampler_xxx_0 = filter.rational_resampler_ccc(
                interpolation=1,
                decimation=dec,
                taps=None,
                fractional_bw=None,
        )
        self.low_pass_filter_0 = filter.fir_filter_ccf(1, firdes.low_pass(
        	1, samp_rate, subport/8, 10e3, firdes.WIN_BLACKMAN, 6.76))
        self.digital_costas_loop_cc_0 = digital.costas_loop_cc(6.28/100.0, 2)
        self.blocks_throttle_0 = blocks.throttle(gr.sizeof_gr_complex*1, samp_rate/dec)
        self.blocks_threshold_ff_0_0_1 = blocks.threshold_ff(0.05, 0.5, 0)
        self.blocks_threshold_ff_0 = blocks.threshold_ff(0.15, 0.4, 0)
        self.blocks_multiply_xx_0_0 = blocks.multiply_vff(1)
        self.blocks_multiply_xx_0 = blocks.multiply_vff(1)
        self.blocks_multiply_const_vxx_0_0 = blocks.multiply_const_vff((1, ))
        self.blocks_multiply_const_vxx_0 = blocks.multiply_const_vcc((8, ))
        self.blocks_float_to_complex_0 = blocks.float_to_complex(1)
        self.blocks_complex_to_real_0_0 = blocks.complex_to_real(1)
        self.blocks_complex_to_mag_0 = blocks.complex_to_mag(1)
        self.blocks_complex_to_float_0 = blocks.complex_to_float(1)

        ##################################################
        # Connections
        ##################################################
        self.connect((self.rtlsdr_source_0, 0), (self.low_pass_filter_0, 0))
        self.connect((self.rational_resampler_xxx_0, 0), (self.blocks_multiply_const_vxx_0, 0))
        self.connect((self.low_pass_filter_0, 0), (self.rational_resampler_xxx_0, 0))
        self.connect((self.blocks_throttle_0, 0), (self.blocks_complex_to_mag_0, 0))
        self.connect((self.blocks_complex_to_mag_0, 0), (self.blocks_threshold_ff_0, 0))
        self.connect((self.blocks_throttle_0, 0), (self.blocks_complex_to_float_0, 0))
        self.connect((self.blocks_multiply_const_vxx_0, 0), (self.blocks_throttle_0, 0))
        self.connect((self.blocks_complex_to_float_0, 0), (self.blocks_multiply_xx_0, 0))
        self.connect((self.blocks_complex_to_float_0, 1), (self.blocks_multiply_xx_0_0, 0))
        self.connect((self.blocks_multiply_xx_0_0, 0), (self.blocks_float_to_complex_0, 1))
        self.connect((self.blocks_multiply_xx_0, 0), (self.blocks_float_to_complex_0, 0))
        self.connect((self.blocks_multiply_const_vxx_0_0, 0), (self.blocks_multiply_xx_0_0, 1))
        self.connect((self.blocks_multiply_const_vxx_0_0, 0), (self.blocks_multiply_xx_0, 1))
        self.connect((self.blocks_threshold_ff_0, 0), (self.blocks_multiply_const_vxx_0_0, 0))
        self.connect((self.blocks_threshold_ff_0_0_1, 0), (self.wxgui_scopesink2_1, 0))
        self.connect((self.blocks_complex_to_real_0_0, 0), (self.wxgui_scopesink2_1, 1))
        self.connect((self.blocks_complex_to_real_0_0, 0), (self.blocks_threshold_ff_0_0_1, 0))
        self.connect((self.rational_resampler_xxx_1, 0), (self.blocks_complex_to_real_0_0, 0))
        self.connect((self.digital_costas_loop_cc_0, 0), (self.rational_resampler_xxx_1, 0))
        self.connect((self.blocks_float_to_complex_0, 0), (self.digital_costas_loop_cc_0, 0))


# QT sink close method reimplementation

    def get_rfid(self):
        return self.rfid

    def set_rfid(self, rfid):
        self.rfid = rfid
        self.set_subport(self.rfid/16)
        self.set_samp_rate(self.rfid/8)
        self.rtlsdr_source_0.set_center_freq(self.rfid+self.upconv+self.subport, 0)

    def get_subport(self):
        return self.subport

    def set_subport(self, subport):
        self.subport = subport
        self.set_sym_rate(self.subport/8)
        self.rtlsdr_source_0.set_center_freq(self.rfid+self.upconv+self.subport, 0)
        self.low_pass_filter_0.set_taps(firdes.low_pass(1, self.samp_rate, self.subport/8, 10e3, firdes.WIN_BLACKMAN, 6.76))

    def get_sym_rate(self):
        return self.sym_rate

    def set_sym_rate(self, sym_rate):
        self.sym_rate = sym_rate
        self.set_samp_per_sym((self.samp_rate/self.dec)/(self.sym_rate))

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.set_samp_per_sym((self.samp_rate/self.dec)/(self.sym_rate))
        self.rtlsdr_source_0.set_sample_rate(self.samp_rate)
        self.low_pass_filter_0.set_taps(firdes.low_pass(1, self.samp_rate, self.subport/8, 10e3, firdes.WIN_BLACKMAN, 6.76))
        self.blocks_throttle_0.set_sample_rate(self.samp_rate/self.dec)
        self.wxgui_scopesink2_1.set_sample_rate(self.samp_rate/self.dec)

    def get_dec(self):
        return self.dec

    def set_dec(self, dec):
        self.dec = dec
        self.set_samp_per_sym((self.samp_rate/self.dec)/(self.sym_rate))
        self.blocks_throttle_0.set_sample_rate(self.samp_rate/self.dec)
        self.wxgui_scopesink2_1.set_sample_rate(self.samp_rate/self.dec)

    def get_upconv(self):
        return self.upconv

    def set_upconv(self, upconv):
        self.upconv = upconv
        self.rtlsdr_source_0.set_center_freq(self.rfid+self.upconv+self.subport, 0)

    def get_samp_per_sym(self):
        return self.samp_per_sym

    def set_samp_per_sym(self, samp_per_sym):
        self.samp_per_sym = samp_per_sym

if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print "Warning: failed to XInitThreads()"
    parser = OptionParser(option_class=eng_option, usage="%prog: [options]")
    (options, args) = parser.parse_args()
    tb = top_block()
    tb.Start(True)
    tb.Wait()

