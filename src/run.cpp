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
  const std::string path_to_traj = "/tmp/ActionTrajectory.csv";
  parser::parseTrajectoryFile(path_to_traj, 12, traj_data);

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

    // Step once to start the controller
    mj_sim->stepSimulation();

    /*
    double d = 10*(value[0] + value[1]);
    for (unsigned int j = 0; j < 3; ++j)
    {
      model.dof_damping[j] = d;
    }
    */

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

    //std::cout << "Value: (" << value[0] << ", " << value[1] << "). Body pos: "<< data.qpos[0] << std::endl;

    std::unique_lock<std::mutex> lock(MTX);

    double score = 0;
    for (unsigned int i = 0; i < traj_data.size(); ++i)
    {
      auto v1 = traj_data[i];
      auto v2 = state_buffer[i];
      score += (euclideanNorm(v1, v2));
    }

    std::cout << "Value: (" << value[0] << ", " << value[1] << "). Score: " << score << std::endl;
    if(score < best_score)
    {
      best_score = score;
      std::cout << "new best score: " << score << "\n";
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
  std::array<double, variables.size()> x0 = {};
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
