SRC_DIR 	:= src/
INC_DIR 	:= inc/
BIN_DIR 	:= bin/

CC			:= g++
CPPFLAGS 	:= -std=c++11 -Wall -O2 -w -Wunused-command-line-argument
CXXFLAGS	:= -I $(INC_DIR)

SRC_FILES 	:= $(wildcard $(SRC_DIR)*.cc)
OBJ_FILES 	:= $(patsubst $(SRC_DIR)%.cc,$(BIN_DIR)%.o,$(SRC_FILES))

OPENCV 		:= `pkg-config --cflags --libs opencv4`

LIBDIR 		:= -L /usr/lib
LIBRARIES	:= $(OPENCV)

DEFAULT_APP := calibrate
APP 		?= $(DEFAULT_APP)

define CC-BIN-BUILD
$(CC) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@ $(LIBDIR) $(LIBRARIES)
endef

all: clean-all mkbin build rmbin

build: $(OBJ_FILES)
	$(CC) $(CPPFLAGS) $^ -o $(APP) $(LIBDIR) $(LIBRARIES)

$(BIN_DIR)%.o: $(SRC_DIR)%.cc 
	$(CC-BIN-BUILD)	

.PHONY: clean clean-all
clean:
	rm -rf $(BIN_DIR)

clean-all: clean
	rm -rf *~ $(SRC_DIR)*~ $(INC_DIR)*~
	rm -rf $(APP) $(DEFAULT_APP)

mkbin:
	mkdir $(BIN_DIR)

rmbin:
	rm -rf $(BIN_DIR)