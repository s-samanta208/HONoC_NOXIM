
#compilation
g++ appSpeTrfONoC.cpp

#RUN

./a.out 4 2>t.txt
cp /home/user122_1/noxim-master_Optical/noxim-master/tables/outGraph19.txt /home/user122_1/noxim-master_Optical/noxim-master/bin/
cd bin
make
./noxim -config ../config_examples/my_config.yaml -optinoc -dimx 8 -dimy 8 -traffic table outGraph19.txt>>Graph19_DS.txt


##### IMPORTANT INSTRUCTION######

	#Text file of Graph/Application and mapping should be in the "mapping_data" folder

	#'4'  -> is the GraphNumber

	#'-dimx 8 -dimy 8' ->  Needs to change it as per the Graph/Application number

	#-optinoc -> To enable the opticalNoC
	
##### IMPORTANT INSTRUCTION ######
