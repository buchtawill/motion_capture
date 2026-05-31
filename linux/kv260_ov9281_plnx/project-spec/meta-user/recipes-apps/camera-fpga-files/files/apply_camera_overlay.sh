#!/bin/sh

/usr/bin/fpgautil -b kv260_ov9281_proj.bit.bin -o mocap-pipeline-overlay.dtbo
/usr/bin/fpgautil -R
/usr/bin/fpgautil -b kv260_ov9281_proj.bit.bin -o mocap-pipeline-overlay.dtbo
