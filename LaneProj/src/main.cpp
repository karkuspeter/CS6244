#include "simple_tui.h"
#include "lane_model.h"

class TUI: public SimpleTUI {
public:
  TUI() {
  }

  DSPOMDP* InitializeModel(option::Option* options) {
    DSPOMDP* model = new LaneModel();
    ((LaneModel*)model)->UnitTest();
    return model;
  }

  void InitializeDefaultParameters() {
	  //Globals::config.pruning_constant = 0.0001;
	  Globals::config.time_per_move = 2;
	  Globals::config.num_scenarios = 500;
  }
};

int main(int argc, char* argv[]) {
  return TUI().run(argc, argv);
}
