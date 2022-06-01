import os, warnings
from pynq import PL
from pynq import Overlay
from pynq import GPIO

overlay=Overlay('/home/xilinx/pynq/overlays/AXI_GPIO/Zynq_wrapper.bit')


class MyOverlay():
	def set_pr(inp):
		if(inp==0):
			overlay.pr_download('pr_hier', '/home/xilinx/pynq/overlays/AXI_GPIO/pr_1.bit')
		else:
			overlay.pr_download('pr_hier', '/home/xilinx/pynq/overlays/AXI_GPIO/pr_2.bit')
    




