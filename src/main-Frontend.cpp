#define EIGEN_NO_DEBUG 1
#define EIGEN_NO_STATIC_ASSERT 1

// Window Version

// C++ Headers

// Project Headers
#include <View/Mainwindow.hpp>

// QT Headers
#include <QApplication>

int main(int argc, char **argv)
{
  QApplication app(argc, argv);
  setlocale(LC_NUMERIC,"C");

  QSurfaceFormat format;
  format.setDepthBufferSize(24);
  format.setSamples(8);
  format.setStencilBufferSize(8);
  format.setProfile(QSurfaceFormat::NoProfile);
  format.setRenderableType(QSurfaceFormat::OpenGL);
  QSurfaceFormat::setDefaultFormat(format);

  MainWindow mainWindow;

  return app.exec();
}