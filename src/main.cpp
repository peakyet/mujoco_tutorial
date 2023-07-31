#include <mujoco/mujoco.h>

class Env {
private:
  mjModel *m;
  mjData *d;

public:
  Env() {
    m = mj_loadXML("./Models/quadrotor_quat.xml", NULL, errstr, errstr_sz);
    d = mj_makeData(m);
  };
  ~Env() {
    mj_deleteModel(m);
    mj_deleteData(d);
  }

  void rollout();
  void controller();
};

void Env::rollout() {
  while (d->time < 10) {
    mj_step(m, d);
  }
}

void Env::controller() {}

int main() {}
