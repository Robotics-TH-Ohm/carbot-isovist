CARBOT_DIR = vendor/Carbot
SIM_LIBS_DIR = $(CARBOT_DIR)/sim_libs
SIM_LIBS = $(SIM_LIBS_DIR)/jts.jar:$(SIM_LIBS_DIR)/commons-math.jar:$(SIM_LIBS_DIR)/opencv-249.jar
SIM_JARS = $(CARBOT_DIR)/jars

AMR_MODULE = dev.vimkat.isovist
SRC_DIR = src

JAVA = java
JAVAC = javac

dev: compile run

compile:
	$(JAVAC) \
		-cp .:$(CARBOT_DIR) \
		-p .:$(SIM_LIBS):$(SIM_JARS)/Basics.jar:$(SIM_JARS)/Customsim.jar:$(SIM_JARS)/Driver.jar:$(SIM_JARS)/Platform.jar:$(SIM_JARS)/Robotinterface.jar:$(SIM_JARS)/Robotlib.jar:$(SIM_JARS)/Simulator.jar \
		--add-modules Robotlib,Robotinterface \
		$(SRC_DIR)/*.java $(SRC_DIR)/isovist/*.java


run:
	$(JAVA) \
		-cp .:$(CARBOT_DIR):$(SRC_DIR)/isovist \
		-p $(CARBOT_DIR)/jars:$(SRC_DIR):$(SIM_LIBS) \
		--add-modules Customsim,Isovist \
		-m Simulator \
		-logdir logs \
		-e $(CARBOT_DIR)/environments/drivedemo.txt \
		-c isovist.DriveDemo \
		-skills -vss,+lss,+lssslam -lidarslam sim  -autorun
