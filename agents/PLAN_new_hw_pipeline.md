** Prompt **
I want to implement a new hardware pipeline based off of the ISP module and blob detector. Right now, there is no ping-pong buffering in hardware, and once a frame is complete, there is a race condition to read the ISP and blob detector data from SW, otherwise data gets overwritten. 

I want you to create a new module, mocap_wrapper, which instantiates the following:
1. the ISP histogram module (instanced as u_isp). Change the RTL so that it buffers the pipeline instead of snooping it
2. the blob detector (instanced as u_blob_detect_rle). 
3. A register file and general control signals / debug info.

Help me brainstorm how to architect this module. Do not take effort into consideration; this pipeline should be architecturally pure. Do not take into consideration what is currently implemented either; just the high level concepts of the ISP and run length encoded blob detector. 

There are two data architectures:
1. SW waits for a HW interrupt, the module asserts an interrupt when data is ready, and SW needs to read data from registers in the HW (256 histogram bins, n blobs detected, cycle counts, etc)
2. SW sets up a DMA buffer and waits for an interrupt, HW streams data as it comes into memory (metadata about the frame - the actual data capture pipeline is still the same), then HW asserts the interrupt when done.

Start by planning an arch / spec for data arch number 1, keeping in mind that transitioning to data arch number 2 is a possibility in the future. The main goal of this brainstorm session is to come up with an architecture that has no race conditions between hardware and software.

** Working Area for agent **
