#include <uWS/uWS.h>
#include <iostream>
#include "particle_filter.h"

/*
int main() {
  uWS::Hub h;
  h.onMessage(
    []
    (
      uWS::WebSocket<uWS::SERVER> ws,
      char* data, size_t length,
      uWS::OpCode opCode
    ) {
      if (length && length > 2 && data[0] == '4' && data[1] == '2')
    }
  ); // end h.onMessage()
}
*/


/*
int main() {
  ParticleFilter pf;
  pf.init(4983, 5029, 1.201, (double[]) {2.0, 2.0, 0.05});
}
*/

#include <random>
#include <map>

int main() {
  std::default_random_engine gen;
  std::vector<double> weights;
  weights.push_back(0.4);
  weights.push_back(0.1);
  weights.push_back(0.4);
  weights.push_back(0.1);
  std::discrete_distribution<> d(weights.begin(), weights.end());
  std::map<int, int> m;
  for(int n = 0; n < 10000; ++n) {
    ++m[d(gen)];
  }
  for(auto p : m) {
    std::cout << p.first << " generated " << p.second << " times\n";
  }
}
