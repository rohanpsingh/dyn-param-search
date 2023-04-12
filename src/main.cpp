#include <mc_rtc/version.h>
#include <mc_rtc/logging.h>

#include <boost/program_options.hpp>

#include <libcmaes/cmaes.h>

#include <chrono>
#include <thread>

#include "run.h"

int main(int argc, char * argv[])
{
  if(mc_rtc::MC_RTC_VERSION != mc_rtc::version())
  {
    mc_rtc::log::error("This program was compiled with {} but mc_rtc is at version {}, you might "
                       "face subtle issues or unexpected crashes, please recompile this program",
                       mc_rtc::MC_RTC_VERSION, mc_rtc::version());
    return 1;
  }

  std::string robot;
  int lambda = -1;
  double sigma = 0.5;
  bool run_once = false;

  boost::program_options::options_description desc("dyn-param-search options");
  // clang-format off
  desc.add_options()
    ("help", "Display help message")
    ("robot", boost::program_options::value<std::string>(&robot)->required(), "Robot module")
    ("lambda", boost::program_options::value<int>(&lambda), "Lambda parameter (offspring per generation")
    ("sigma", boost::program_options::value<double>(&sigma), "Sigma parameter")
    ("run-once", boost::program_options::bool_switch(&run_once), "Run once and exit");
  // clang-format on

  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
  try
  {
    boost::program_options::notify(vm);
  }
  catch(const boost::program_options::required_option & error)
  {
    std::cerr << error.what() << "\n";
    std::cerr << desc << "\n";
    return 1;
  }

  if(vm.count("help"))
  {
    std::cout << desc << "\n";
    return 0;
  }

  optimizer::mute_mc_rtc();

  optimizer::set_main_robot(robot);
  auto bounds = bounds_from_safety();

  std::array<double, variables.size()> init = optimizer::get_x0();
  std::array<double, variables.size()> lbounds;
  std::array<double, variables.size()> ubounds;
  for(size_t i = 0; i < init.size(); ++i)
  {
    const auto & c = bounds.at(variables[i]);
    lbounds[i] = c.min;
    ubounds[i] = c.max;
  }

  if(run_once)
  {
    //optimizer::set_best_score(0.0);
    auto start_t = std::chrono::high_resolution_clock::now();
    double score = optimizer::run(init.data());
    std::chrono::duration<double, std::milli> dt = std::chrono::high_resolution_clock::now() - start_t;
    optimizer::unmute_mc_rtc();
    mc_rtc::log::info("Took {:.2f} ms to run the plan, score: {}", dt.count(), score);
    return 0;
  }

  libcmaes::GenoPheno<libcmaes::pwqBoundStrategy> gp(lbounds.data(), ubounds.data(), variables.size());
  libcmaes::CMAParameters<decltype(gp)> cmaparams(init.size(), init.data(), sigma, lambda, 0, gp);
  // sets elitism: 0 -> no elitism 1 -> elitism: reinjects the best-ever seen solution 2 -> initial elitism: reinject x0
  // as long as it is not improved upon 3 -> initial elitism on restart: restart if best encountered solution is not the
  // the final solution and reinjects the best solution until the population has better fitness, in its majority
  cmaparams.set_elitism(1);
  cmaparams.set_algo(aCMAES);
  cmaparams.set_mt_feval(true);
  cmaparams.set_fplot("cmaes_out.dat");
  libcmaes::FitFunc fit_fun = [](const double * x, const int) { return optimizer::run(x); };
  libcmaes::CMASolutions cmasols = libcmaes::cmaes<decltype(gp)>(fit_fun, cmaparams);
  Eigen::VectorXd params = gp.pheno(cmasols.get_best_seen_candidate().get_x_dvec());
  std::cout << "Optimization time: " << cmasols.elapsed_time() / 1000.0 << " seconds\n";
  std::cout << "Evaluations: " << cmasols.fevals() << "\n";
  std::cout << "Final params: " << params.transpose() << std::endl;
  std::cout << "Best score: " << cmasols.get_best_seen_candidate().get_fvalue() << std::endl;

  return 0;
}
