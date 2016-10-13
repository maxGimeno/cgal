#ifndef PQISOTROPICREMESHINGWIDGET_H
#define PQISOTROPICREMESHINGWIDGET_H
#include "pqPropertyWidget.h"
#include <QTextEdit>
#include <QDebug>
class customTextEdit : public QTextEdit
{
  Q_OBJECT
  Q_PROPERTY(QString text2 READ toPlainText WRITE setText)
public:
  customTextEdit(const QString& s, QWidget* parent=0)
    :QTextEdit(s, parent)
  {
  }
  ~customTextEdit(){}
};
class pqIsotropicRemeshingWidget : public pqPropertyWidget
{
  Q_OBJECT
  typedef pqPropertyWidget Superclass;
public:
  pqIsotropicRemeshingWidget(
    vtkSMProxy *smproxy, vtkSMProperty *smproperty, QWidget *parentObject=0);
  virtual ~pqIsotropicRemeshingWidget();
private:
  customTextEdit* bbox;

  Q_DISABLE_COPY(pqIsotropicRemeshingWidget)
};

#endif // PQISOTROPICREMESHINGWIDGET_H
