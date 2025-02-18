/**
 * @file Raptor.cpp
 * @brief Raptor class implementation
 *
 * This file contains the implementation of the Raptor class, which represents
 * the Round-Based Public Transit Routing algorithm, for journey planning.
 *
 * @autor Maria
 * @date 10/28/2024
 */
#include "Raptor.h"

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
    const std::string &id1 = it1->first;  // Get stop_id
    Stop &stop1 = it1->second;  // Get stop itself

    // Start the inner loop from the next element
    for (auto it2 = std::next(it1); it2 != stops_.end(); ++it2) {
      const std::string &id2 = it2->first; // Get stop_id
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
  // Format departure time to HH:MM:00
  std::ostringstream time_oss;
  time_oss << std::setw(2) << std::setfill('0') << query_.departure_time.hours << ":"
           << std::setw(2) << std::setfill('0') << query_.departure_time.minutes << ":00";

  // Print query details
  std::cout << "Query from " << stops_[query_.source_id].getField("stop_name")
            << " to " << stops_[query_.target_id].getField("stop_name")
            << " departing " << query_.date.day << "/" << query_.date.month << "/" << query_.date.year
            << " (" << weekdays_names[query_.date.weekday]
            << ") at " << time_oss.str() << std::endl << std::endl;

  // Initialize data structures
  arrivals_.clear();
  prev_marked_stops.clear();
  marked_stops.clear();

  // Initialize arrival times for all stops
  for (const auto &[id, stop]: stops_)
    arrivals_[id] = std::vector<StopInfo>(1, {std::nullopt, std::nullopt, std::nullopt, std::nullopt});

  // Initialize the round 0
  k = 0;
  markStop(query_.source_id, Utils::timeToSeconds(query_.departure_time), std::nullopt, std::nullopt);

  k++; // k=1

  // Fill active trips for current and next day
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
    // Get service calendar for the trip
    const Calendar &calendar = calendars_.at(trip.getField("service_id"));

    // Check if the service is active for the target date
    // Only active services are considered
    if (isServiceActive(calendar, target_date))
      trip.setActive(day, true);
    else
      trip.setActive(day, false);
  }
}


std::vector<Journey> Raptor::findJourneys() {
  std::vector<Journey> journeys;

  initializeAlgorithm();

  while (true) {

    // Print round number
    std::cout << std::endl << "Round " << k << std::endl << std::endl;

    // Set upper bound for arrival times
    // Use the minimum arrival time from the previous round as the base for the current round
    setUpperBound();

    // Update previous marked stops and clear current marked stops
    // Only focus on active stops from the previous round
    prev_marked_stops = marked_stops;
    marked_stops.clear();

    // Accumulate routes serving marked stops from previous round
    // ((route_id, direction_id), stop_id)
    // routes_stops_set:
    // {
    //   {(route_id, direction_id), stop_id},
    //   {(route_id, direction_id), stop_id},
    //   ...
    // }
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

      Journey journey = reconstructJourney();

      if (isValidJourney(journey)) {
        journeys.push_back(journey);

        std::cout << "Found journey with " << journey.steps.size() << " step(s)." << std::endl;
        Raptor::showJourney(journey);
      }
    }

    k++;
  }

  size_t before_filtering = journeys.size();

  // Keep only pareto-optimal journeys
  keepParetoOptimal(journeys);

  size_t after_filtering = journeys.size();

  if (before_filtering > after_filtering)
    std::cout << "Discarded " << before_filtering - after_filtering << " journeys." << std::endl;
  return journeys;
}

void Raptor::setUpperBound() {
  // Use the minimum arrival time from the previous round as the base for the current round
  for (const auto &[stop_id, stop]: stops_)
    setMinArrivalTime(stop_id, arrivals_[stop_id][k - 1]);
}

// Define the type of routes_stops_set
std::unordered_set<std::pair<std::pair<std::string, std::string>, std::string>, nested_pair_hash>
Raptor::accumulateRoutesServingStops() {
  std::unordered_set<std::pair<std::pair<std::string, std::string>, std::string>, nested_pair_hash> routes_stops_set;

  // For each previously marked stop p
  for (const auto &marked_stop_id: prev_marked_stops) {

    if (marked_stop_id == query_.target_id) continue; // No need to accumulate routes serving the target stop

    // For each route r serving p
    // Route key: (route_id, direction_id)
    for (const auto &route_key: stops_[marked_stop_id].getRouteKeys()) {

      // Initialize a flag to check if the route already has a point in the list
      // Make sure that max. one stop is added per route
      bool already_has_point = false;
      // Iterate over all stops in the route
      for (std::string stop_id: routes_[route_key].getStopsIds()) {

        // Check if a point p' is already in the list
        // If not, existing_entry is routes_stops_set.end()
        auto existing_entry = routes_stops_set.find({route_key, stop_id});
        // For route r, if a point p' is already in the list, then we need to check if marked_p comes before p'
        // Only the first stop of a route is added to the list
        if (existing_entry != routes_stops_set.end()) {
          // Avoid adding the same stop twice
          already_has_point = true;

          // Find the position of marked_p in the route
          auto it_marked = std::find_if(routes_[route_key].getStopsIds().begin(),
                                        routes_[route_key].getStopsIds().end(),
                                        [&](const std::string &s_id) { return s_id == marked_stop_id; });
          // Find the position of p' in the route
          auto it_stop = std::find_if(routes_[route_key].getStopsIds().begin(), routes_[route_key].getStopsIds().end(),
                                      [&](const std::string &s_id) { return s_id == stop_id; });

          if (it_marked < it_stop) { // if marked_p comes before p'
            routes_stops_set.erase(existing_entry);
            routes_stops_set.insert({route_key, marked_stop_id});
          } // else (if marked_p does not come before p'), we do not add it to the set and leave the p' entry
        }
      }

      // If the route does not have a point in the list, then add the stop
      if (!already_has_point)
        routes_stops_set.insert({route_key, marked_stop_id});
      // TODO: avoid accumulating routes that do not have any active trip
    }
  }

  // Return the set of routes serving stops
  return routes_stops_set;
}

void Raptor::traverseRoutes(
        std::unordered_set<std::pair<std::pair<std::string, std::string>, std::string>, nested_pair_hash> routes_stops_set) {
  
  // Iterate over all routes in the set
  auto route_stop = routes_stops_set.begin();
  while (route_stop != routes_stops_set.end()) {
    // Get the route key and the stop id
    const auto &[route_key, p_stop_id] = *route_stop;
    // Get the route
    const Route &route = routes_[route_key];
    // Find the position of the stop in the route
    auto stop_it = std::find_if(route.getStopsIds().begin(), route.getStopsIds().end(),
                                [&](const std::string &s_id) {
                                  return s_id == p_stop_id;
                                });

    // For each stop pi on this route, try to find the earliest trip (et) that can be taken
    // Iterate over all stops in the route after the stop p
    for (auto it = stop_it; it != route.getStopsIds().end(); ++it) {
      // Get the stop id
      std::string pi_stop_id = *it;

      // Get the arrival time of the stop in the previous round k-1
      std::optional<int> stop_prev_arrival = arrivals_[pi_stop_id][k - 1].arrival_seconds; // TODO: really k-1?

      // If stop is not reachable in the previous round k-1, no trip can be caught
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

  std::optional<Day> stop_day = arrivals_[pi_stop_id][k - 1].day;
  std::optional<int> stop_prev_arrival = arrivals_[pi_stop_id][k - 1].arrival_seconds;

  // If stop is not reachable, no trip can be caught
  if (!stop_day.has_value()) return std::nullopt;

  // Find the earliest trip in route r that can be caught at stop pi in round k
  for (const auto &stop_time_key: stops_[pi_stop_id].getStopTimesKeys()) {
    const StopTime &stop_time = stop_times_.at(stop_time_key);

    if (earlier(stop_time.getDepartureSeconds(),
                stop_prev_arrival)) // If departure time is earlier than arrival
      continue;

    if (isValidTrip(route_key, stop_time, stop_day.value()))
      // We can return because a stop's stop_times is ordered
      return std::make_pair(stop_time.getField("trip_id"), stop_day.value());
  }

  // If no trip was found for the current day, try the next day
  for (const auto &stop_time_key: stops_[pi_stop_id].getStopTimesKeys()) {
    const StopTime &stop_time = stop_times_.at(stop_time_key);

    if ((stop_day.value() == Day::NextDay)
        && earlier(stop_time.getDepartureSeconds(),
                   stop_prev_arrival)) // If departure time is earlier than arrival
      continue;

    if (isValidTrip(route_key, stop_time, Day::NextDay))
      // We can return because a stop's stop_times is ordered
      return std::make_pair(stop_time.getField("trip_id"), Day::NextDay);
  }

  return std::nullopt;
}

bool Raptor::isValidTrip(const std::pair<std::string, std::string> &route_key,
                         const StopTime &stop_time, const Day &day) {

  const std::string &trip_id = stop_time.getField("trip_id");
  const Trip &trip = trips_[trip_id];

  auto [route_id, direction_id] = route_key;

  // Check if the stop_time is from a trip that belongs to the route being traversed
  if ((trip.getField("route_id") != route_id) || (trip.getField("direction_id") != direction_id))
    return false;

  int departure_secs = day == Day::CurrentDay ? stop_time.getDepartureSeconds()
                                              : stop_time.getDepartureSeconds() + MIDNIGHT;
  std::optional<int> stop_prev_arrival = arrivals_[stop_time.getField("stop_id")][k - 1].arrival_seconds;
  std::optional<int> target_arrival = arrivals_[query_.target_id][k].arrival_seconds;

  if (trip.isActive(day)
      && !earlier(departure_secs, stop_prev_arrival) // Does not depart earlier than stop's arrival
      && earlier(departure_secs, target_arrival)) // Departs earlier than target's arrival
    return true;

  return false;
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

//  if ((stop_id == query_.target_id)) {
//    std::string day_str = Utils::dayToString(day);
//    std::cout << "Marking " << stop_id << " at " << Utils::secondsToTime(arrival) << day_str <<
//              " ptrip " << parent_trip_id.value_or("foot") << " from stop "
//              << parent_stop_id.value_or("none") << " in which we arrive at "
//              << Utils::secondsToTime(arrivals_[parent_stop_id.value()][k].arrival_seconds.value());
//    std::cout << std::endl;
//  }
}

// Updates arrival time of stops that are connected by footpaths
void Raptor::handleFootpaths() {
  // For each previously marked stop p
  for (const auto &stop_id: prev_marked_stops) {

    // If parent step is a footpath, then do not check further footpaths, in order to avoid approximation errors
    if (isFootpath(arrivals_[stop_id][k - 1])) continue;

    std::optional<int> p_prev_arrival = arrivals_[stop_id][k - 1].arrival_seconds;

    // For each footpath (p, p')
    for (const auto &[dest_id, duration]: stops_[stop_id].getFootpaths()) {
      int new_arrival = p_prev_arrival.value() + duration;
      if (dest_id == query_.target_id && stop_id == "OTH1")
        std::cout << "from " << stop_id << " to " << dest_id << " with duration " << Utils::secondsToTime(duration)
                  << " departing at " << Utils::secondsToTime(p_prev_arrival.value())
                  << " and arriving at " << Utils::secondsToTime(new_arrival) << std::endl;

      if (improvesArrivalTime(new_arrival, dest_id))
        markStop(dest_id, new_arrival, std::nullopt, stop_id);

    } // end each footpath (p, p')
  } // end each marked stop p

}

bool Raptor::isFootpath(const StopInfo &stop_info) {
  return stop_info.parent_stop_id.has_value() && !stop_info.parent_trip_id.has_value();
}

Journey Raptor::reconstructJourney() {
  Journey journey;
  std::string current_stop_id = query_.target_id;

  while (true) {

    const std::optional<std::string> parent_trip_id_opt = arrivals_[current_stop_id][k].parent_trip_id;
    std::optional<std::string> parent_agency_name = std::nullopt;

    const std::optional<std::string> parent_stop_id_opt = arrivals_[current_stop_id][k].parent_stop_id;

    if (!parent_stop_id_opt.has_value()) break;

    const std::string &parent_stop_id = parent_stop_id_opt.value();

    int departure_seconds, duration, arrival_seconds;
    if (!parent_trip_id_opt.has_value()) { // Footpath
      arrival_seconds = arrivals_[current_stop_id][k].arrival_seconds.value();
      duration = stops_[parent_stop_id].getFootpaths().at(current_stop_id);
      departure_seconds = arrival_seconds - duration;

    } else { // Trip
      const std::string &parent_trip_id = parent_trip_id_opt.value();
      std::string route_id = trips_[parent_trip_id].getField("route_id");

      std::string agency_id;
      for (const auto &[key, route]: routes_) {
        // Check if the first part of the key (route_id) matches
        if (key.first == route_id) {
          agency_id = route.getField("agency_id");
          parent_agency_name = agencies_.at(agency_id).getField("agency_name");
          break;
        }
      }

      departure_seconds = stop_times_[{parent_trip_id, parent_stop_id}].getDepartureSeconds();
      arrival_seconds = arrivals_[current_stop_id][k].arrival_seconds.value();
      duration = arrival_seconds - departure_seconds;
    }

    Day day = arrival_seconds > MIDNIGHT ? Day::NextDay : Day::CurrentDay;
    JourneyStep step = {parent_trip_id_opt, parent_agency_name, &stops_[parent_stop_id], &stops_[current_stop_id],
                        departure_seconds, day, duration, arrival_seconds};

    journey.steps.push_back(step);

    // Update to the previous stop boarded
    current_stop_id = parent_stop_id;
  }

  // Reverse the journey to obtain the correct sequence
  std::reverse(journey.steps.begin(), journey.steps.end());

  // Set journey departure secs and day
  journey.departure_secs = journey.steps.front().departure_secs;
  journey.departure_day = journey.steps.front().day;

  // Set journey arrival secs and day
  journey.arrival_secs = journey.steps.back().arrival_secs;
  journey.arrival_day = journey.steps.back().day;

  // Set journey duration
  journey.duration = journey.arrival_secs - journey.departure_secs;

  return journey;
}

bool Raptor::isValidJourney(Journey journey) const {
  if (journey.steps.empty() ||
      (journey.steps.front().src_stop->getField("stop_id") != query_.source_id))
    return false;

  return true;
}

const std::unordered_map<std::string, Stop> &Raptor::getStops() const {
  return stops_;
}

void Raptor::showJourney(const Journey &journey) {

  // Print the header row
  std::cout << std::setw(5) << "step" << std::setw(8) << "day"
            << std::setw(10) << "dep_time " << std::setw(8) << "stop " << std::setw(14) << "(name)"
            << std::setw(10) << "duration "
            << std::setw(8) << "-> stop " << std::setw(14) << "(name)" << std::setw(9) << "arr_time "
            << std::setw(14) << " trip "
            << std::setw(7) << "agency" << std::endl;

  // Print the journey steps
  for (int j = 0; j < journey.steps.size(); j++) {
    const JourneyStep &step = journey.steps[j];

    std::cout << std::setw(5) << j + 1;

    std::string day = Utils::dayToString(step.day);
    std::cout << std::setw(8) << day;

    std::cout << std::setw(10) << Utils::secondsToTime(step.departure_secs)
              << std::setw(8) << step.src_stop->getField("stop_id")
              << std::setw(14) << Utils::getFirstWord(step.src_stop->getField("stop_name"))
              << std::setw(10) << Utils::secondsToTime(step.duration)
              << std::setw(8) << step.dest_stop->getField("stop_id")
              << std::setw(14) << Utils::getFirstWord(step.dest_stop->getField("stop_name"))
              << std::setw(10) << Utils::secondsToTime(step.arrival_secs);

    if (step.trip_id.has_value()) {
      std::cout << std::setw(13) << step.trip_id.value();
      std::cout << std::setw(7) << Utils::getFirstWord(step.agency_name.value());
    } else
      std::cout << std::setw(12) << "footpath";


    std::cout << std::endl << std::endl;
  }
}

bool Raptor::isDominatedByAny(const std::vector<Journey> &journeys, const Journey &journey) {
  return std::ranges::any_of(journeys, [&journey](const Journey &otherJourney) {
    return dominates(otherJourney, journey);
  });
}

void Raptor::keepParetoOptimal(std::vector<Journey> &journeys) {
  journeys.erase(std::remove_if(journeys.begin(), journeys.end(),
                                [&journeys](const Journey &journey) {
                                  return isDominatedByAny(journeys, journey);
                                }),
                 journeys.end());
}

// Keeps only the Pareto-optimal journeys
bool Raptor::dominates(const Journey &journey1, const Journey &journey2) {
  // If two journeys have the same number of steps, keep the one with the earliest arrival time
  // If a journey has more steps but not better arrival time, discard it
  return ((journey1.duration < journey2.duration
           && journey1.steps.size() <= journey2.steps.size())
          || (journey1.steps.size() < journey2.steps.size()
              && journey1.duration <= journey2.duration));
}
