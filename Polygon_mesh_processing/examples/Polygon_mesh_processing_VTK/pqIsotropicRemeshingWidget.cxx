#include "pqIsotropicRemeshingWidget.h"

#include "pqPropertiesPanel.h"
#include <QGridLayout>
#include <QLabel>


pqIsotropicRemeshingWidget::pqIsotropicRemeshingWidget(
    vtkSMProxy *smproxy, vtkSMProperty *smproperty, QWidget *parentObject)
    : Superclass(smproxy, parentObject)
    {
      this->setShowLabel(false);
      QGridLayout* gridLayout = new QGridLayout(this);
      gridLayout->setMargin(pqPropertiesPanel::suggestedMargin());
      gridLayout->setHorizontalSpacing(pqPropertiesPanel::suggestedHorizontalSpacing());
      gridLayout->setVerticalSpacing(pqPropertiesPanel::suggestedVerticalSpacing());
      gridLayout->setColumnStretch(1, 0);
      QLabel* customLabel = new QLabel("Bbox coordinates", this);
      gridLayout->addWidget(customLabel);

      bbox = new customTextEdit(QString("min(0,0,0)\n max(0,0,0)"), this);
      bbox->setMaximumHeight(5*QFontMetrics(bbox->font()).height());
      bbox->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::MinimumExpanding);
      bbox->setObjectName("BBox");
      bbox->setReadOnly(true);
      gridLayout->addWidget(bbox);

      this->addPropertyLink(bbox, "text2", SIGNAL(textChanged()), smproperty);
      this->setChangeAvailableAsChangeFinished(true);
    }
pqIsotropicRemeshingWidget::~pqIsotropicRemeshingWidget()
{
}
