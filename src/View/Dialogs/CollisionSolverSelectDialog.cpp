
#include "CollisionSolverSelectDialog.hpp"

// QT Headers
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

// Public

void CollisionSolverSelectDialog::setupSelectDialog(const std::vector<std::string>& methods, QComboBox* chooser, QTimeEdit* timeBox){
  timeBox->setParent(this);
  setWindowModality(Qt::WindowModal);
  QLabel* chooserLabel = new QLabel("Method: ", this);
  QLabel* secondLabel = new QLabel("Maximum time (hh:mm:ss): ", this);
  QVBoxLayout* mainLayout = new QVBoxLayout(this);
  QHBoxLayout* chooserLayout = new QHBoxLayout();
  QHBoxLayout* secondLayout = new QHBoxLayout();
  QVBoxLayout* contentLayout = new QVBoxLayout();
  QHBoxLayout* buttonLayout = new QHBoxLayout();
  QPushButton* okButton = new QPushButton(QString("OK"), this);
  QPushButton* cancelButton = new QPushButton(QString("Cancel"), this);
  for(const std::string& method: methods){
    chooser->addItem(QString(method.c_str()));
  }
  chooserLayout->addWidget(chooserLabel);
  chooserLayout->addWidget(chooser);

  secondLayout->addWidget(secondLabel);

  timeBox->setMinimumTime(QTime(0, 0, 0, 0));
  timeBox->setDisplayFormat(QString("hh:mm:ss"));
  timeBox->setSelectedSection(QDateTimeEdit::SecondSection);
  secondLayout->addWidget(timeBox);

  contentLayout->addLayout(chooserLayout);
  contentLayout->addLayout(secondLayout);

  okButton->connect(okButton, SIGNAL(released()), this, SLOT(accept()));
  cancelButton->connect(cancelButton, SIGNAL(released()), this, SLOT(reject()));
  buttonLayout->addStretch();
  buttonLayout->addWidget(okButton);
  buttonLayout->addWidget(cancelButton);

  mainLayout->addLayout(contentLayout);
  mainLayout->addLayout(buttonLayout);
  setLayout(mainLayout);
  setWindowTitle(QString("Choose collisionsolver."));
  chooser->setFocus();
}

void CollisionSolverSelectDialog::setupSelectDialog(const std::pair<std::vector<std::string>, std::vector<std::string>>& methods, QComboBox* costChooser, QComboBox* placementChooser, QTimeEdit* timeBox){
  timeBox->setParent(this);
  setWindowModality(Qt::WindowModal);
  QLabel* costLabel = new QLabel("Cost: ", this);
  QLabel* placementLabel = new QLabel("Placement: ", this);
  QLabel* secondLabel = new QLabel("Maximum time (hh:mm:ss): ", this);
  QVBoxLayout* mainLayout = new QVBoxLayout(this);
  QVBoxLayout* labelLayout = new QVBoxLayout();
  QVBoxLayout* chooserLayout = new QVBoxLayout();
  QHBoxLayout* secondLayout = new QHBoxLayout();
  QHBoxLayout* firstLayout = new QHBoxLayout();
  QVBoxLayout* contentLayout = new QVBoxLayout();
  QHBoxLayout* buttonLayout = new QHBoxLayout();
  QPushButton* okButton = new QPushButton(QString("OK"), this);
  QPushButton* cancelButton = new QPushButton(QString("Cancel"), this);
  for(const std::string& method: methods.first){
    costChooser->addItem(QString(method.c_str()));
  }
  for(const std::string& method: methods.second){
    placementChooser->addItem(QString(method.c_str()));
  }
  labelLayout->addWidget(costLabel);
  labelLayout->addWidget(placementLabel);
  chooserLayout->addWidget(costChooser);
  chooserLayout->addWidget(placementChooser);

  firstLayout->addLayout(labelLayout);
  firstLayout->addLayout(chooserLayout);

  secondLayout->addWidget(secondLabel);
  timeBox->setMinimumTime(QTime(0, 0, 0, 0));
  timeBox->setDisplayFormat(QString("hh:mm:ss"));
  timeBox->setSelectedSection(QDateTimeEdit::SecondSection);
  secondLayout->addWidget(timeBox);

  contentLayout->addLayout(firstLayout);
  contentLayout->addLayout(secondLayout);

  okButton->connect(okButton, SIGNAL(released()), this, SLOT(accept()));
  cancelButton->connect(cancelButton, SIGNAL(released()), this, SLOT(reject()));
  buttonLayout->addStretch();
  buttonLayout->addWidget(okButton);
  buttonLayout->addWidget(cancelButton);

  mainLayout->addLayout(contentLayout);
  mainLayout->addLayout(buttonLayout);
  setLayout(mainLayout);
  setWindowTitle(QString("Choose collisionsolver."));
  costChooser->setFocus();
}