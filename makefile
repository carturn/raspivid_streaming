build:
	g++ *.cpp *.c -I/opt/vc/include/ -I/opt/vc/include/interface/vcos/pthreads/ -I/opt/vc/include/interface/vmcs_host/linux/ -L/opt/vc/lib/ -lmmal_core -lmmal -lmmal_util -lmmal_components -lmmal_vc_client -lbcm_host -lvcos -lvcsm -lvchiq_arm -lvchostif -lvcilcs -lcontainers -lvcfiled_check -lkhrn_client -fpermissive
