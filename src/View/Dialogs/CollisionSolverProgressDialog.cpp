
#include "CollisionSolverProgressDialog.hpp"

// QT Headers
#include <QLabel>
#include <QString>

// Public

void CollisionSolverProgressDialog::setupProgressDialog(int time){
  setWindowTitle("Solving...");
  setLabel(new QLabel(QString("Solving collisions: "), this));
  setRange(0, static_cast<int>(time));
  setWindowModality(Qt::WindowModal);
  setMinimumDuration(0);
  setValue(0);
}
