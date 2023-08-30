#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "interface.h"
#include "model.h"

typedef bool (*function_t)(void);

enum Menu {
  kIntroduction = 0,
  kPrimary,
  kCompletion,
  kMenuCount
};

enum Function {
  kPrimaryFuncs = 0,
  kFunctionCount
};

enum FunctionCount {
  kPrimaryFuncsCount = 2
};

static function_t* functions[kFunctionCount] = {};

static const char* const m_menus[kMenuCount] = {
    " -------------------------------------------------------------- \n"
    "|                  MechatronicArmAnalyzer 1.0                  |\n"
    " -------------------------------------------------------------- \n",
    " -------------------------------------------------------------- \n"
    "|                       Select menu item                       |\n"
    " -------------------------------------------------------------- \n"
    "| 0. Exit                                                      |\n"
    "| 1. Forward Kinematics six axis dh-model                      |\n"
    " -------------------------------------------------------------- \n"
    " > ",
    " -------------------------------------------------------------- \n"
    "|            Successful completion of the programme            |\n"
    " -------------------------------------------------------------- \n"};

static int8_t check_input_item(const int8_t min, const int8_t max) {
  char line[256];
  fgets(line, sizeof(line), stdin);

  int8_t result;
  while (!sscanf(line, "%hhd", &result) || result <= min || result >= max) {
    printf("Incorrect input, try again: ");
    fgets(line, sizeof(line), stdin);
  }

  return result;
}

static size_t show_menu(const char* menu, const size_t items) {
  printf(menu);
  return items ? check_input_item(-1, items) : 0;
}

static bool run_menu(const function_t* func, size_t size, size_t menu) {
  bool flag = false;
  size_t item = 0;
  do {
    item = show_menu(m_menus[menu], size);
  } while (size && (flag = func[item]()));

  return !flag;
}

static bool close() {
  return false;
}

static bool position() {
  static double theta[POINT_DIMENSION] = {};
  for (int i = 0; i < POINT_DIMENSION; ++i) {
    printf("theta[%d]: ", i);
    if (scanf("%lf", &theta[i]) != 1) {
      printf("Incorrect input, try again: ");
      --i;
    }
    getchar();
  }
  point_t point = get_end_effector_position(theta);
  printf("Position end-effector {\n\tx: %+3.4f\n\ty: %+3.4f\n\tz: %+3.4f\n}\n", point.x, point.y, point.z);

  return true;
}

static void initialize() {
  functions[kPrimaryFuncs] = (function_t*)malloc(sizeof(function_t) * kPrimaryFuncsCount);
  functions[kPrimaryFuncs][0] = close;
  functions[kPrimaryFuncs][1] = position;
}

static void cleanup() {
  for (int idx = 0; idx < kFunctionCount; ++idx) {
    free(functions[idx]);
  }
}

void interface() {
  show_menu(m_menus[kIntroduction], 0);
  initialize();

  run_menu(functions[kPrimaryFuncs], kPrimaryFuncsCount, kPrimary);

  cleanup();
  show_menu(m_menus[kCompletion], 0);
}
