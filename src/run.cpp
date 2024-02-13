#include "run.h"

#include <mc_mujoco/mj_sim.h>
#include <boost/filesystem.hpp>

namespace optimizer
{

static std::mutex MTX;

static std::string main_robot;
static double best_score = std::numeric_limits<double>::infinity();

static bool render;

static std::string joint_name;

std::map<std::string, std::vector<double>> init_qs_;
std::map<std::string, sva::PTransformd> init_pos_;

std::vector<std::vector<double>> traj_data;


std::string toLowerCase(const std::string& input)
{
  std::string result = input;
  for (char& c : result) {
    c = std::tolower(c);
  }

  return result;
}

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
    config.add("Enabled", "JointCalibController");
    config.save(temp_conf.string());
  }

  mc_mujoco::MjConfiguration mj_config;
  mj_config.fix_base_link = true;
  if(!render)
  {
    mj_config.with_visualization = false;
  }
  mj_config.mc_config = temp_conf.string();

  auto sim = std::make_unique<mc_mujoco::MjSim>(mj_config);
  boost::filesystem::remove(temp_conf);

  // parse trajectory file
  const std::string path_to_traj = "/tmp/real_" + joint_name + ".csv";
  try
  {
    traj_data = parser::parseTrajectoryFile(path_to_traj, 1);
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    std::terminate();
  }

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
    std::string mj_jnt_name = toLowerCase(main_robot) + "_" + joint_name;
    value_to_model(mj_jnt_name, value, model);

    // Step once to start the controller
    mj_sim->stepSimulation();


    // get motor mbc id
    int mbc_idx = controller.robot().jointIndexByName(joint_name);

    double wallclock = 0;
    bool done = false;
    int counter = 0;
    std::vector<std::vector<double>> state_buffer;

    while(!done)
    {
      if (render && (counter % 20)==0)
      {
        mj_sim->updateScene();
        mj_sim->render();
      }
      mj_sim->stepSimulation();

      if((counter)%5==0)
      {
	std::vector<double> v = controller.realRobot().mbc().q[mbc_idx];
	state_buffer.push_back(v);
      }
      wallclock += model.opt.timestep;
      counter++;

      done = (state_buffer.size() >= traj_data.size());
    }

    try
    {
      if(state_buffer.size()!=traj_data.size())
      {
        throw std::runtime_error("Trajectory data must have the same size. (Parsed traj = " + std::to_string(traj_data.size()) + ". Collected traj size = " + std::to_string(state_buffer.size()) + ").");
      }
    }
    catch(std::exception const& e)
    {
      std::cout << "Exception: " << e.what() << "\n";
    }

    std::unique_lock<std::mutex> lock(MTX);

    std::vector<double> p1;
    std::vector<double> p2;
    for (const auto& v : traj_data)
    {
      p1.push_back(v[0]);
    }
    for (const auto& v : state_buffer)
    {
      p2.push_back(v[0]);
    }
    double score = euclideanNorm(p1, p2);

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

  std::string mj_jnt_name = toLowerCase(main_robot) + "_" + joint_name;
  model_to_value(mj_jnt_name, model, x0.data());
  return x0;
}

void set_best_score(double s)
{
  best_score = s;
}

void set_render()
{
  render = true;
}

void set_main_robot(const std::string & robot, const std::string & joint)
{
  main_robot = robot;
  joint_name = joint;
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
