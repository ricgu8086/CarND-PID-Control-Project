#include <uWS/uWS.h>
#include <iostream>
#include <vector>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi()
{
	return M_PI;
}
double deg2rad(double x)
{
	return x * pi() / 180;
}
double rad2deg(double x)
{
	return x * 180 / pi();
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s)
{
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_last_of("]");
	if (found_null != std::string::npos)
	{
		return "";
	}
	else if (b1 != std::string::npos && b2 != std::string::npos)
	{
		return s.substr(b1, b2 - b1 + 1);
	}
	return "";
}

int main()
{
	uWS::Hub h;

	PID pid;
	// : Initialize the pid variable.

	double Kp = 1;
	double Ki = 1;
	double Kd = 1;

	pid.Init(Kp, Ki, Kd);

	/* TWIDDLE algoritm */
	bool twiddle_on = true;

	vector<double> p = {0, 0, 0};
	vector<double> dp = {1, 1, 1};

	double best_error = 0;
	double error = 0, accumulated_error = 0;
	double sum_dp = 0;
	int warmup_iterations = 100, total_iterations = 2*warmup_iterations, curr_iterations = 0;
	enum class State {BEGINNING, BLOCK1, BLOCK2, FINISHED}; // The meaning of this enum is explained in PID.cpp in PID::Twiddle
	State curr_state = State::BEGINNING;
	int i = 0;
	double tol = 1e-2;
	/* /TWIDDLE algoritm */

	h.onMessage(
			[&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
			{
				// "42" at the start of the message means there's a websocket message event.
				// The 4 signifies a websocket message
				// The 2 signifies a websocket event
				if (length && length > 2 && data[0] == '4' && data[1] == '2')
				{
					auto s = hasData(std::string(data).substr(0, length));
					if (s != "")
					{
						auto j = json::parse(s);
						std::string event = j[0].get<std::string>();
						if (event == "telemetry")
						{

							// j[1] is the data JSON object
							double cte = std::stod(j[1]["cte"].get<std::string>());
							double speed = std::stod(j[1]["speed"].get<std::string>());
							double angle = std::stod(j[1]["steering_angle"].get<std::string>());
							double steer_value;
							/*
							 * : Calculate steering value here, remember the steering value is
							 * [-1, 1].
							 * NOTE: Feel free to play around with the throttle and speed. Maybe use
							 * another PID controller to control the speed!
							 */

							/* TWIDDLE algoritm */

							if(twiddle_on)
							{
								curr_iterations += 1;

								if(curr_iterations >= warmup_iterations)
								{
									accumulated_error += cte*cte;
								}

								if(curr_iterations == total_iterations) // A cycle is completed
								{
									curr_iterations = 0;
									error = accumulated_error/(double)warmup_iterations;
									accumulated_error = 0;

									switch(curr_state)
									{
										case State::BEGINNING :
											best_error = error;

											sum_dp = 0;

											for(auto n: dp)
												sum_dp += n;

											if (sum_dp > tol)
											{
												i = 0;
												p[i] += dp[i];
												pid.Init(p); // Reset controller state
												curr_state = State::BLOCK1;
											}

											else
												curr_state = State::FINISH;

											break;

										case State::BLOCK1 :

											if (error < best_error)
											{
												best_error = error;
												dp[i] *= 1.1;

												i += 1

												if (i == p.size())
												{
													sum_dp = 0;

													for(auto n: dp)
															sum_dp += n;

													if (sum_dp > tol)
													{
														i = 0;
														p[i] += dp[i];
														pid.Init(p); // Reset controller state
														curr_state = State::BLOCK1;
													}
													else
														curr_state = State::FINISHED;
												}
											}
											else
											{
												p[i] -= 2*dp[i];
												pid.Init(p); // Reset controller state
												curr_state = State::BLOCK2;
											}

											break;

										case State::BLOCK2 :

											if(error < best_error)
											{
												best_error = error;
												dp[i] *= 1.1;
											}
											else
											{
												p[i] += dp[i];
												dp[i] *= 0.9;
											}

											i += 1

											if (i == p.size())
											{
												sum_dp = 0;

												for(auto n: dp)
														sum_dp += n;

												if (sum_dp > tol)
												{
													i = 0;
													p[i] += dp[i];
													pid.Init(p); // Reset controller state
													curr_state = State::BLOCK1;
												}
												else
													curr_state = State::FINISHED;
											}

											break;
									}

									if (curr_state == State::FINISHED)
									{
										cout << "Parameter tuning accomplished" << endl;
										cout << "Params: " << p[0] << ", " << p[1] << ", " << p[2] << endl;
										cout << "Best error: " best_error;

										return 1;
									}

								}
							}

							/* /TWIDDLE algoritm */

							pid.UpdateError(cte);
							steer_value = pid.TotalError();

							// Clipping steer_value to [-1, 1]
							if (steer_value > 1)
								steer_value = 1;
							if (steer_value < -1)
								steer_value = -1;

							// DEBUG
							std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

							json msgJson;
							msgJson["steering_angle"] = steer_value;
							msgJson["throttle"] = 0.3;
							auto msg = "42[\"steer\"," + msgJson.dump() + "]";
							std::cout << msg << std::endl;
							ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
						}
					}
					else
					{
						// Manual driving
						std::string msg = "42[\"manual\",{}]";
						ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
					}
				}
			});

	// We don't need this since we're not using HTTP but if it's removed the program
	// doesn't compile :-(
	h.onHttpRequest(
			[](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t)
			{
				const std::string s = "<h1>Hello world!</h1>";
				if (req.getUrl().valueLength == 1)
				{
					res->end(s.data(), s.length());
				}
				else
				{
					// i guess this should be done more gracefully?
					res->end(nullptr, 0);
				}
			});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
	{
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection(
			[&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
			{
				ws.close();
				std::cout << "Disconnected" << std::endl;
			});

	int port = 4567;
	if (h.listen(port))
	{
		std::cout << "Listening to port " << port << std::endl;
	}
	else
	{
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}