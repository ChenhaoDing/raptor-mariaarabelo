
#include "Raptor.h"

#include <utility>


Raptor::Raptor(const std::unordered_map<std::string, Agency> &agencies,
               const std::unordered_map<std::string, Calendar> &calendars,
               const std::unordered_map<std::string, Stop> &stops,
               const std::unordered_map<std::pair<std::string, std::string>, Route, pair_hash> &routes,
               const std::unordered_map<std::string, Trip> &trips,
               const std::unordered_map<std::pair<std::string, std::string>, StopTime, pair_hash> &stop_times)
        : agencies_(agencies), calendars_(calendars), stops_(stops), routes_(routes), trips_(trips),
          stop_times_(stop_times) {
  k = 1;

  std::cout << "Raptor initialized with "
            << agencies_.size() << " agencies, "
            << calendars_.size() << " calendars, "
            << stops.size() << " stops, "
            << routes.size() << " routes, "
            << trips.size() << " trips and "
            << stop_times.size() << " stop times." << std::endl;

  initializeFootpaths();
}

void Raptor::setQuery(const Query &query) {
  query_ = query;
}

void Raptor::initializeFootpaths() {
  // Initialize footpaths
  std::cout << "Initializing footpaths..." << std::endl;
  auto start_time = std::chrono::high_resolution_clock::now();

  // Avoid duplicating calculations for both sides
  for (auto it1 = stops_.begin(); it1 != stops_.end(); ++it1) {
    const std::string &id1 = it1->first;
    Stop &stop1 = it1->second;

    // Start the inner loop from the next element
    for (auto it2 = std::next(it1); it2 != stops_.end(); ++it2) {
      const std::string &id2 = it2->first; // stop_id
      Stop &stop2 = it2->second; // stop itself

      // Calculate duration between the two stops
      int duration = Utils::getDuration(
              stop1.getField("stop_lat"), stop1.getField("stop_lon"),
              stop2.getField("stop_lat"), stop2.getField("stop_lon"));

      // Add footpaths in both directions
      stop1.addFootpath(id2, duration);
      stop2.addFootpath(id1, duration);
    }
  }

  auto end_time = std::chrono::high_resolution_clock::now();

  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

  std::cout << "Footpaths initialized in " << duration << " ms ("
            << duration / 1000 << " seconds)." << std::endl;
}

void Raptor::initializeAlgorithm() {
  std::ostringstream time_oss;
  time_oss << std::setw(2) << std::setfill('0') << query_.departure_time.hours << ":"
           << std::setw(2) << std::setfill('0') << query_.departure_time.minutes << ":00";

  std::cout << "Finding journeys from " << stops_[query_.source_id].getField("stop_name")
            << " to " << stops_[query_.target_id].getField("stop_name")
            << " departing " << query_.date.day << "/" << query_.date.month << "/" << query_.date.year
            << " (" << weekdays_names[query_.date.weekday] << ") "
            << " at " << time_oss.str() << std::endl << std::endl;

  arrivals_.clear();
  prev_marked_stops.clear();
  marked_stops.clear();

  for (const auto &[id, stop]: stops_)
    arrivals_[id] = std::vector<StopInfo>(1, {std::nullopt, std::nullopt, std::nullopt, std::nullopt});

  k = 0;
  markStop(query_.source_id, Utils::timeToSeconds(query_.departure_time), std::nullopt, std::nullopt);

  k++; // k=1

  fillActiveTrips(Day::CurrentDay);
  fillActiveTrips(Day::NextDay);
}

void Raptor::setMinArrivalTime(const std::string &stop_id, StopInfo stop_info) {

  if (arrivals_[stop_id].size() <= k)
    arrivals_[stop_id].resize(k + 1);

  arrivals_[stop_id][k] = std::move(stop_info);

}

void Raptor::fillActiveTrips(Day day) {
  Date target_date = (day == Day::CurrentDay) ? query_.date : Utils::addOneDay(query_.date);

  // Iterates over all trips
  for (auto &[trip_id, trip]: trips_) {
    const Calendar &calendar = calendars_.at(trip.getField("service_id"));

    if (isServiceActive(calendar, target_date))
      trip.setActive(day, true);
    else
      trip.setActive(day, false);
  }
}


std::vector<std::vector<JourneyStep>> Raptor::findJourneys() {
  std::vector<std::vector<JourneyStep>> journeys;

  initializeAlgorithm();
  std::cout << "ftpth tcrz2 sct2" << Utils::secondsToTime(stops_["TCRZ2"].getFootpaths().at("SCT2")) << std::endl;
  std::cout << "ftpth tcrz2 mtcr2" << Utils::secondsToTime(stops_["TCRZ2"].getFootpaths().at("MTCR2")) << std::endl;

  while (true) {

    std::cout << std::endl << "Round " << k << std::endl << std::endl;

    // 1st: set an upper bound for the current round
    for (const auto &[stop_id, stop]: stops_)
      setMinArrivalTime(stop_id, arrivals_[stop_id][k - 1]);

    prev_marked_stops = marked_stops;
    marked_stops.clear();

    // Accumulate routes serving marked stops from previous round
    // ((route_id, direction_id), stop_id)
    std::unordered_set<std::pair<std::pair<std::string, std::string>, std::string>, nested_pair_hash> routes_stops_set = accumulateRoutesServingStops();
    std::cout << "Accumulated " << routes_stops_set.size() << " routes serving stops." << std::endl;

    // 2nd: Traverse each route
    traverseRoutes(routes_stops_set);
    std::cout << "Traversed routes. " << marked_stops.size() << " stop(s) improved." << std::endl;

    // Look for footpaths
    handleFootpaths();
    std::cout << "Handled footpaths. " << marked_stops.size() << " stop(s) improved." << std::endl;

    // Stopping criterion: if no stops are marked, then stop
    if (marked_stops.empty()) break;

    if (marked_stops.find(query_.target_id) != marked_stops.end()) {
      std::cout << "Target improved! Reconstructing journey..." << std::endl;

      std::vector<JourneyStep> journey = reconstructJourney();

      if (isValidJourney(journey)) {
        journeys.push_back(journey);

        std::cout << "Found journey with " << journey.size() << " step(s)." << std::endl;
        Raptor::showJourney(journey);
      }
    }

    k++;
  }

  // Keep only pareto-optimal journeys
  // If two journeys have the same number of steps, keep the one with the earliest arrival time
  // If a journey has more steps but worse arrival time, discard it

  return journeys;
}

std::unordered_set<std::pair<std::pair<std::string, std::string>, std::string>, nested_pair_hash>
Raptor::accumulateRoutesServingStops() {
  std::unordered_set<std::pair<std::pair<std::string, std::string>, std::string>, nested_pair_hash> routes_stops_set;

  // For each previously marked stop p
  for (const auto &marked_stop_id: prev_marked_stops) {

    if (marked_stop_id == query_.target_id) continue; // No need to accumulate routes serving the target stop

    // For each route r serving p
    for (const auto &route_key: stops_[marked_stop_id].getRouteKeys()) {
      // TODO: avoid accumulating routes that do not have any active trip

      // Iterate over all stops in the route
      bool already_has_point = false;
      for (std::string stop_id: routes_[route_key].getStopsIds()) {

        // Check if a point p' is already in the list
        auto existing_entry = routes_stops_set.find({route_key, stop_id});
        if (existing_entry != routes_stops_set.end()) {
          already_has_point = true;

          auto it_marked = std::find_if(routes_[route_key].getStopsIds().begin(),
                                        routes_[route_key].getStopsIds().end(),
                                        [&](const std::string &s_id) { return s_id == marked_stop_id; });
          auto it_stop = std::find_if(routes_[route_key].getStopsIds().begin(), routes_[route_key].getStopsIds().end(),
                                      [&](const std::string &s_id) { return s_id == stop_id; });

          if (it_marked < it_stop) { // if marked_p comes before p'
            routes_stops_set.erase(existing_entry);
            routes_stops_set.insert({route_key, marked_stop_id});
          } // else (if marked_p does not come before p'), we do not add it to the set and leave the p' entry
        }
      }

      if (!already_has_point)    // The route does not have a point in the list
        routes_stops_set.insert({route_key, marked_stop_id});
    }
  }

  return routes_stops_set;
}

void Raptor::traverseRoutes(
        std::unordered_set<std::pair<std::pair<std::string, std::string>, std::string>, nested_pair_hash> routes_stops_set) {
  auto route_stop = routes_stops_set.begin();

  while (route_stop != routes_stops_set.end()) {
    const auto &[route_key, p_stop_id] = *route_stop;
    const Route &route = routes_[route_key];

    auto stop_it = std::find_if(route.getStopsIds().begin(), route.getStopsIds().end(),
                                [&](const std::string &s_id) {
                                  return s_id == p_stop_id;
                                });

    // For each stop pi on this route, try to find the earliest trip (et) that can be taken
    for (auto it = stop_it; it != route.getStopsIds().end(); ++it) {
      std::string pi_stop_id = *it;

      // If stop is not reachable, no trip can be caught
      std::optional<int> stop_prev_arrival = arrivals_[pi_stop_id][k - 1].arrival_seconds; // TODO: really k-1?

      if (!stop_prev_arrival.has_value()) continue;

      // Find the earliest trip in route r that can be caught at stop pi in round k
      auto et = findEarliestTrip(pi_stop_id, route_key);

      // If a valid trip was found, traverse the trip
      if (et.has_value()) {
        std::string et_id = et.value().first;
        Day et_day = et.value().second;
        traverseTrip(et_id, et_day, pi_stop_id);
      } else continue; // No valid trip found for this stop

    } // end each stop pi on route

    route_stop = routes_stops_set.erase(route_stop);
  } // end each route

}

std::optional<std::pair<std::string, Day>>
Raptor::findEarliestTrip(const std::string &pi_stop_id, const std::pair<std::string, std::string> &route_key) {

  // Find the earliest trip in route r that can be caught at stop pi in round k
  for (const auto &stop_time_key: stops_[pi_stop_id].getStopTimesKeys()) {
    // TODO: start checking from where stop time departure >= Tk-1(pi)
    const StopTime &stop_time = stop_times_.at(stop_time_key);

    auto [valid_trip, closest_day] = isValidTrip(route_key, stop_time);
    if (valid_trip)
      // We can return because a stop's stop_times is ordered
      return std::make_pair(stop_time.getField("trip_id"), closest_day.value());
  }

  return std::nullopt;
}

std::pair<bool, std::optional<Day>>
Raptor::isValidTrip(const std::pair<std::string, std::string> &route_key,
                    const StopTime &stop_time) {

  const std::string &trip_id = stop_time.getField("trip_id");
  const Trip &trip = trips_[trip_id];

  auto [route_id, direction_id] = route_key;

  // Check if the stop_time is from a trip that belongs to the route being traversed
  if ((trip.getField("route_id") != route_id) || (trip.getField("direction_id") != direction_id))
    return {false, std::nullopt};

  std::optional<Day> stop_day = arrivals_[stop_time.getField("stop_id")][k - 1].day;
  std::optional<int> stop_prev_arrival = arrivals_[stop_time.getField("stop_id")][k - 1].arrival_seconds;

  int departure_secs = stop_time.getDepartureSeconds();
  int next_day_departure_secs = departure_secs + MIDNIGHT;
  std::optional<int> target_arrival = arrivals_[query_.target_id][k].arrival_seconds;

  // If stop is today reachable
  if ((stop_day.value() == Day::CurrentDay)
      && trip.isActive(Day::CurrentDay)
      && !earlier(departure_secs, stop_prev_arrival) // Does not depart earlier than stop's arrival
      && earlier(departure_secs, target_arrival)) // Departs earlier than target's arrival
    return {true, Day::CurrentDay}; // trip current day departs >= stop's arrival time


  // Either trip was not active in the current day, or the stop is only next day reachable
  if ((trip.isActive(Day::NextDay))
      && !earlier(next_day_departure_secs, stop_prev_arrival) // Does not depart earlier than stop's arrival
      && earlier(next_day_departure_secs, target_arrival)) // Departs earlier than target's arrival
    return {true, Day::NextDay}; // trip next day departs >= stop's earliest arrival time


  return {false, std::nullopt};

}

bool Raptor::isServiceActive(const Calendar &calendar, const Date &date) {

  // Check if the date is within the calendar's start and end dates
  if (!Utils::dateWithinRange(date, calendar.getField("start_date"), calendar.getField("end_date")))
    return false;

  // Check if the day of the week is active
  return std::stoi(calendar.getField(weekdays_names[date.weekday]));
}

// TODO: create a struct for stop_time_key
// TODO: use .at() instead of []
void Raptor::traverseTrip(std::string &et_id, Day &et_day, std::string &pi_stop_id) {
  Trip et = trips_.at(et_id);

  auto et_stop_it = std::find_if(et.getStopTimesKeys().begin(), et.getStopTimesKeys().end(),
                                 [&](const std::pair<std::string, std::string> &st_key) {
                                   return st_key.second == pi_stop_id;
                                 });

  // Traverse remaining stops on the trip to update arrival times
  for (auto next_stop_time_key = std::next(et_stop_it);
       next_stop_time_key != et.getStopTimesKeys().end(); ++next_stop_time_key) {

    auto [_, next_stop_id] = *next_stop_time_key;
    StopTime &next_stop_time = stop_times_[*next_stop_time_key];

    // Access arrival seconds at next_stop_id for trip et_id, according to the day
    int arr_secs = et_day == Day::CurrentDay ? next_stop_time.getArrivalSeconds()
                                             : next_stop_time.getArrivalSeconds() + MIDNIGHT;

    // If arrival time can be improved, update Tk(pj) using et
    if (improvesArrivalTime(arr_secs, next_stop_id))
      markStop(next_stop_id, arr_secs, et_id, pi_stop_id);

    // Check if an earlier trip can be caught at stop i (because a quicker path was found in a previous round)
    if ((arrivals_[next_stop_id][k - 1].parent_trip_id.has_value()) // Because we can instantly arrive at source
        && (arrivals_[next_stop_id][k - 1].arrival_seconds < arr_secs))  // if Tk-1(pi) < Tarr(t, pi)
      break;

  } // end remaining stops on trip et_id

}

bool Raptor::earlier(int secondsA, std::optional<int> secondsB) {
  if (!secondsB.has_value()) return true; // if still not set, then any value is better
  return secondsA < secondsB.value();
}

bool Raptor::improvesArrivalTime(int arrival, const std::string &dest_id) {
  return earlier(arrival, arrivals_[dest_id][k].arrival_seconds) // Required
         && earlier(arrival, arrivals_[query_.target_id][k].arrival_seconds); // Pruning
}

void Raptor::markStop(const std::string &stop_id, int arrival,
                      const std::optional<std::string> &parent_trip_id,
                      const std::optional<std::string> &parent_stop_id) {
  Day day = arrival > MIDNIGHT ? Day::NextDay : Day::CurrentDay;
  setMinArrivalTime(stop_id, {arrival, parent_trip_id, parent_stop_id, day});
  marked_stops.insert(stop_id);

  if ((stop_id == query_.target_id) || (stop_id == "MTCR2")) {
    std::string day_str = day == Day::NextDay ? " next day " : " current day ";
    std::cout << "Marking " << stop_id << " at " << Utils::secondsToTime(arrival) << day_str <<
              " ptrip " << parent_trip_id.value_or("none") << " from stop " << parent_stop_id.value_or("none");
    if (parent_stop_id.has_value())
      std::cout << " in which we arrive at "
                << Utils::secondsToTime(arrivals_[parent_stop_id.value()][k].arrival_seconds.value());

    std::cout << std::endl;
  }
}


void Raptor::handleFootpaths() {
  // For each previously marked stop p
  for (const auto &stop_id: prev_marked_stops) {

    // If parent step is a footpath, then do not check further footpaths, in order to avoid approximation errors
    if (isFootpath(arrivals_[stop_id][k - 1])) continue;

    std::optional<int> p_prev_arrival = arrivals_[stop_id][k - 1].arrival_seconds;

    // For each footpath (p, p')
    for (const auto &[dest_id, duration]: stops_[stop_id].getFootpaths()) {
      int new_arrival = p_prev_arrival.value() + duration;

      if (improvesArrivalTime(new_arrival, dest_id)) {
        // Only updates footpath if it goes to a stop that has not made any improvements
//          std::cout << "Marking " << std::setw(10) << dest_id << "fp from " << std::setw(10) << stop_id
//                    << " " << Utils::secondsToTime(arrivals_[stop_id][k].arrival_seconds) << " + "
//                    << Utils::secondsToTime(footpath.duration) << " = " << Utils::secondsToTime(new_arrival)
//                    << " < min(" << Utils::secondsToTime(arrivals_[dest_id][k].arrival_seconds) << ", "
//                    << Utils::secondsToTime(arrivals_[query_.target_id][k].arrival_seconds) << ")" << std::endl;
        markStop(dest_id, new_arrival, std::nullopt, stop_id);
      }

    } // end each footpath (p, p')
  } // end each marked stop p

}

bool Raptor::isFootpath(const StopInfo &stop_info) {
  return stop_info.parent_stop_id.has_value() && !stop_info.parent_trip_id.has_value();
}

std::vector<JourneyStep> Raptor::reconstructJourney() {
  std::vector<JourneyStep> journey;
  std::string current_stop_id = query_.target_id;

  std::cout << "Reconstructing journey at k " << k << std::endl;

  while (true) {

    const std::optional<std::string> parent_trip_id_opt = arrivals_[current_stop_id][k].parent_trip_id;
    const std::optional<std::string> parent_stop_id_opt = arrivals_[current_stop_id][k].parent_stop_id;

    if (!parent_stop_id_opt.has_value()) break;

    const std::string &parent_stop_id = parent_stop_id_opt.value();

    int departure_seconds, duration, arrival_seconds;
    if (!parent_trip_id_opt.has_value()) { // Footpath
      departure_seconds = arrivals_[parent_stop_id][k].arrival_seconds.value();
      duration = stops_[parent_stop_id].getFootpaths().at(current_stop_id);
      arrival_seconds = departure_seconds + duration;
    } else { // Trip
      const std::string &parent_trip_id = parent_trip_id_opt.value();
      departure_seconds = stop_times_[{parent_trip_id, parent_stop_id}].getDepartureSeconds();
      arrival_seconds = arrivals_[current_stop_id][k].arrival_seconds.value();
      duration = arrival_seconds - departure_seconds;
    }

    JourneyStep step = {parent_trip_id_opt, &stops_[parent_stop_id], &stops_[current_stop_id],
                        departure_seconds, duration, arrival_seconds};
    journey.push_back(step);
    std::cout << "Added step from " << parent_stop_id << " to " << current_stop_id << std::endl;

    // Update to the previous stop boarded
    current_stop_id = parent_stop_id;
  }

  // Reverse the journey to obtain the correct sequence
  std::reverse(journey.begin(), journey.end());

  return journey;
}

bool Raptor::isValidJourney(std::vector<JourneyStep> journey) const {
  if (journey.empty() ||
      (journey.front().src_stop->getField("stop_id") != query_.source_id))
    return false;

  // TODO: will be possible to remove it?
//  for (size_t i = 0; i < journey.size() - 1; ++i)
//    if (!journey[i].trip_id.has_value() && !journey[i + 1].trip_id.has_value())
//      return false;

  return true;
}

const std::unordered_map<std::string, Stop> &Raptor::getStops() const {
  return stops_;
}

void Raptor::showJourney(const std::vector<JourneyStep> &journey) {

  // Print the header row
  std::cout << std::setw(5) << "step" << std::setw(7) << "day"
            << std::setw(13) << " trip "
            << std::setw(8) << "stop " << std::setw(15) << "(name)" << std::setw(10) << "dep_time "
            << std::setw(10) << "duration "
            << std::setw(14) << "-> stop " << std::setw(15) << "(name)" << std::setw(9) << "arr_time " << std::endl;

  // Print the journey steps
  for (int j = 0; j < journey.size(); j++) {
    const JourneyStep &step = journey[j];

    std::cout << std::setw(5) << j + 1;

    std::string day = step.departure_secs > MIDNIGHT ? "next" : "current";
    std::cout << std::setw(8) << day;

    if (step.trip_id.has_value())
      std::cout << std::setw(12) << step.trip_id.value();
    else
      std::cout << std::setw(12) << "footpath";

    std::cout << std::setw(8) << step.src_stop->getField("stop_id") << std::setw(15)
              << Utils::getFirstWord(step.src_stop->getField("stop_name")) << std::setw(10)
              << Utils::secondsToTime(step.departure_secs)
              << std::setw(10) << Utils::secondsToTime(step.duration)
              << std::setw(14) << step.dest_stop->getField("stop_id") << std::setw(15)
              << Utils::getFirstWord(step.dest_stop->getField("stop_name")) << std::setw(9)
              << Utils::secondsToTime(step.arrival_secs);

    std::cout << std::endl << std::endl;
  }

}



