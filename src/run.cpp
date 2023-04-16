#include "run.h"

#include <mc_mujoco/mj_sim.h>
#include <boost/filesystem.hpp>

namespace optimizer
{

static std::mutex MTX;

static std::string main_robot;
static double best_score = std::numeric_limits<double>::infinity();

bool render = false;

std::map<std::string, std::vector<double>> init_qs_;
std::map<std::string, sva::PTransformd> init_pos_;

std::vector<std::vector<double>> traj_data;

static std::unique_ptr<mc_mujoco::MjSim> make_sim()
{
  boost::filesystem::path temp_conf = boost::filesystem::unique_path("/tmp/dyn-param-search-%%%%-%%%%-%%%%-%%%%.yaml");
  {
    mc_rtc::Configuration config;
    if(main_robot.size())
    {
      config.add("MainRobot", main_robot);
    }
    config.add("ClearGlobalPluginPath", true);
    config.add("Plugins", std::vector<std::string>{});
    config.add("IncludeHalfSitController", false);
    config.add("Log", false);
    config.add("GUIServer").add("Enable", false);
    config.add("Enabled", "WalkerPolicy");
    config.save(temp_conf.string());
  }

  mc_mujoco::MjConfiguration mj_config;
  if(!render)
  {
    mj_config.with_visualization = false;
  }
  mj_config.mc_config = temp_conf.string();

  auto sim = std::make_unique<mc_mujoco::MjSim>(mj_config);
  boost::filesystem::remove(temp_conf);

  // parse trajectory file
  const std::string path_to_traj = "/tmp/StateTrajectory.csv";
  traj_data = parser::parseTrajectoryFile(path_to_traj, 12);

  // populate init q and pos
  auto & gc = *sim->controller();
  auto & controller = gc.controller();
  std::string rname = controller.robot().name();
  for(const auto & r : controller.robots())
  {
    init_qs_[r.name()] = controller.robot(r.name()).encoderValues();
    init_pos_[r.name()] = controller.robot(r.name()).posW();
  }
  return sim;
}

static std::vector<std::unique_ptr<mc_mujoco::MjSim>> pool;
static std::mutex pool_mutex;

static std::unique_ptr<mc_mujoco::MjSim> get_sim()
{
  std::unique_lock<std::mutex> lock(pool_mutex);
  if(pool.empty())
  {
    lock.unlock();
    return make_sim();
  }
  else
  {
    auto * sim = pool.back().release();
    pool.erase(pool.begin() + (pool.size() - 1));
    lock.unlock();
    sim->resetSimulation(init_qs_, init_pos_);
    return std::unique_ptr<mc_mujoco::MjSim>(sim);
  }
}

static void release_sim(std::unique_ptr<mc_mujoco::MjSim> && sim)
{
  std::unique_lock<std::mutex> lock(pool_mutex);
  pool.push_back(std::move(sim));
}

double run(const double * value)
{
  try
  {
    auto mj_sim = get_sim();
    auto & gc = *mj_sim->controller();
    auto & controller = gc.controller();

    auto model = mj_sim->model();
    auto data = mj_sim->data();

    // set simulation parameters
    value_to_model(value, model);

    // Step once to start the controller
    mj_sim->stepSimulation();

    double wallclock = 0;
    bool done = false;
    int counter = 0;
    std::vector<std::vector<double>> state_buffer;
    while(!done)
    {
      if(controller.datastore().has("AUTOPLAY_FINISH"))
      {
        done = controller.datastore().get<bool>("AUTOPLAY_FINISH");
      }
      if (render && (counter % 20)==0)
      {
        mj_sim->updateScene();
        mj_sim->render();
      }
      mj_sim->stepSimulation();

      if(controller.datastore().has("AUTOPLAY_FINISH") && ((counter+1)%2==0))
      {
        auto v = controller.datastore().get<std::vector<double>>("AUTOPLAY_BUFFER");
        if (v.size())
        {
          state_buffer.push_back(v);
        }
      }
      wallclock += model.opt.timestep;
      counter++;
    }

    try
    {
      if(state_buffer.size()!=traj_data.size())
      {
        throw std::runtime_error("Trajectory data must have the same size");
      }
    }
    catch(std::exception const& e)
    {
      std::cout << "Exception: " << e.what() << "\n";
    }

    std::unique_lock<std::mutex> lock(MTX);

    std::vector<double> norms;
    double ts = controller.timeStep;
    for (unsigned int i = 0; i < traj_data.size(); ++i)
    {
      auto p1 = traj_data[i];
      auto p2 = state_buffer[i];
      std::vector<double> v1, v2;
      for (unsigned int j = 0; j < p1.size(); ++j)
      {
        double _v1 = 0;
        double _v2 = 0;
        if(i > 0)
        {
          _v1 = (traj_data[i][j] - traj_data[i-1][j])/ts;
          _v2 = (state_buffer[i][j] - state_buffer[i-1][j])/ts;
        }
        v1.push_back(_v1);
        v2.push_back(_v2);
      }
      std::vector<double> x(p1);
      std::vector<double> y(p2);
      x.insert(x.end(), v1.begin(), v1.end());
      y.insert(y.end(), v2.begin(), v2.end());
      norms.push_back(euclideanNorm(x, y));
    }
    double score = 0;
    for (const auto & n : norms)
    {
      score += n;
    }
    score /= norms.size();

    std::cout << "Value: (";
    for (unsigned int i = 0; i < variables.size(); ++i)
    {
      std::cout << value[i] << ",";
    }
    std::cout << "). Score: " << score << std::endl;

    if(score < best_score)
    {
      best_score = score;
      std::cout << "new best score: " << score << "\n";
      {
        unsigned int j = norms.size()-1;
        std::cout << j << "------->Norm:" << norms[j] << std::endl;
      }
      std::cout << std::endl;
    }
    lock.unlock();

    release_sim(std::move(mj_sim));

    return score;
  }
  catch(const std::runtime_error &)
  {
    return 1e12;
  }
}

std::array<double, variables.size()> get_x0()
{
  std::array<double, variables.size()> x0;
  auto mj_sim = get_sim();
  auto model = mj_sim->model();
  model_to_value(model, x0.data());
  return x0;
}

void set_best_score(double s)
{
  best_score = s;
}

void set_main_robot(const std::string & robot)
{
  main_robot = robot;
}

void mute_mc_rtc()
{
  mc_rtc::log::details::success().set_level(spdlog::level::off);
  mc_rtc::log::details::info().set_level(spdlog::level::off);
  mc_rtc::log::details::cerr().set_level(spdlog::level::off);
}

void unmute_mc_rtc()
{
  mc_rtc::log::details::success().set_level(spdlog::level::debug);
  mc_rtc::log::details::info().set_level(spdlog::level::debug);
  mc_rtc::log::details::cerr().set_level(spdlog::level::debug);
}

double euclideanNorm(std::vector<double> v1, std::vector<double> v2)
{
  if (v1.size() != v2.size())
  {
    throw std::runtime_error("Vectors must have the same size");
  }
  std::vector<double> diff(v1.size());
  for (int i = 0; i < v1.size(); i++)
  {
    diff[i] = v1[i] - v2[i];
  }
  double norm = 0.0;
  for (int i = 0; i < diff.size(); i++)
  {
    norm += pow(diff[i], 2);
  }
  return sqrt(norm);
};

} // namespace optimizer
