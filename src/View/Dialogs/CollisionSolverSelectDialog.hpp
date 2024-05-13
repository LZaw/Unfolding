
#pragma once

// C++ Headers
#include <string>
#include <vector>

// QT Headers
#include <QComboBox>
#include <QDialog>
#include <QTimeEdit>

class CollisionSolverSelectDialog: public QDialog{
public:
  using QDialog::QDialog;

  void setupSelectDialog(const std::vector<std::string>& methods, QComboBox* chooser, QTimeEdit* timeBox);
  void setupSelectDialog(const std::pair<std::vector<std::string>, std::vector<std::string>>& methods, QComboBox* costChooser, QComboBox* placementChooser, QTimeEdit* timeBox);
};
