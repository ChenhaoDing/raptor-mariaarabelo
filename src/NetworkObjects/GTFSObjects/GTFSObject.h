//
// Created by maria on 11/20/2024.
//

#ifndef RAPTOR_GTFSOBJECT_H
#define RAPTOR_GTFSOBJECT_H

#include <unordered_map>
#include <string>
#include <stdexcept>
#include <functional>

#include "../../Utils.h"

class GTFSObject {
public:
  // Set a field value
  virtual void setField(const std::string& field, const std::string& value);

  // Get a field value
  virtual const std::string& getField(const std::string& field) const;

  const std::unordered_map<std::string, std::string> &getFields() const;
protected:
  std::unordered_map<std::string, std::string> fields; // Dynamic fields

};


#endif //RAPTOR_GTFSOBJECT_H
