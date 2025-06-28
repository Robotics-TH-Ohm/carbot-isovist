CARBOT_DIR = vendor/Carbot
SIM_LIBS_DIR = $(CARBOT_DIR)/sim_libs
SIM_LIBS = $(SIM_LIBS_DIR)/jts.jar:$(SIM_LIBS_DIR)/commons-math.jar:$(SIM_LIBS_DIR)/opencv-249.jar
SIM_JARS = $(CARBOT_DIR)/jars

SRC_DIR = Robotcontroller

JAVA = java
JAVAC = javac

CARBOTSIM = $(JAVA) \
	-cp .:$(CARBOT_DIR) \
	-p .:$(CARBOT_DIR)/jars:$(SIM_LIBS) \
	--add-modules Customsim,Robotcontroller \
	-m Simulator \
	-logdir logs \
	-envscale 0.25 \
	-e environments/sonsbeek.txt \
	-cfg isovist-grid.xml \

dev: compile run-localization

compile:
	$(JAVAC) \
		-cp .:$(CARBOT_DIR) \
		-p .:$(SIM_LIBS):$(SIM_JARS)/Basics.jar:$(SIM_JARS)/Customsim.jar:$(SIM_JARS)/Driver.jar:$(SIM_JARS)/Platform.jar:$(SIM_JARS)/Robotinterface.jar:$(SIM_JARS)/Robotlib.jar:$(SIM_JARS)/Simulator.jar \
		--add-modules Robotlib,Robotinterface \
		$(shell find $(SRC_DIR) -name *.java)

run-localization:
	$(CARBOTSIM) \
		-c robotcontroller.LocalizationController \
		-skills -vss,+lss,+lssslam -lidarslam sim -autorun

run-mapping:
	$(CARBOTSIM) \
		-c robotcontroller.MappingController \
		-skills -vss,+lss,+lssslam -lidarslam sim -autorun
