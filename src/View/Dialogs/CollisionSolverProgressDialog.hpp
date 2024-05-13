
#pragma once

// QT Headers
#include <QProgressDialog>
#include <QWidget>

class CollisionSolverProgressDialog: public QProgressDialog{
public:
  using QProgressDialog::QProgressDialog;

  void setupProgressDialog(int time);
};
