//
// Created by maria on 11/11/2024.
//

#ifndef RAPTOR_APPLICATION_H
#define RAPTOR_APPLICATION_H

#include "Raptor.h"
#include <iostream>
#include <iomanip>

class Application {
public:
  explicit Application(std::string  inputDirectory);
  void run();

private:
  static void handleQuery(Raptor &raptor);
  static void showCommands();

  std::string inputDirectory;
};

#endif //RAPTOR_APPLICATION_H
