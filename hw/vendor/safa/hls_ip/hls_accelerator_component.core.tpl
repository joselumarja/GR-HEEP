CAPI=2:

name: uclm:arco:hls_accelerator_component:1.0.0
description: HLS streaming accelerator component (1 fifo input, 1 fifo output, hs control signals)

filesets:
  rtl:
    files:
@HLS_RTL_FILES@
    file_type: verilogSource

targets:
  default:
    filesets:
    - rtl
    toplevel: @HLS_TOP_MODULE@