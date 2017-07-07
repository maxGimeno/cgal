#define foreach Q_FOREACH
#include <dtkBRep>
#include <dtkNurbsSurface>

#include "Scene_cad_item.h"
#include "Scene_nurbs_item.h"
#include <QDebug>

struct Scene_cad_item_priv{
  Scene_cad_item_priv(dtkBRep* brep, CGAL::Three::Scene_interface* scene, Scene_cad_item* parent)
    :m_brep(brep), item(parent)
  {
    std::size_t i = 0;
    const std::vector < dtkNurbsSurface* > nurbs_surfaces = m_brep->nurbsSurfaces();

    for (auto it = nurbs_surfaces.begin(); it != nurbs_surfaces.end(); ++it) {
    Scene_nurbs_item* nurbs_item =  new Scene_nurbs_item(*(*it));
    nurbs_item->setFlatMode();
    nurbs_item->setName(QString("Nurbs #%1").arg(i));
    scene->addItem(nurbs_item);
    item->addChild(nurbs_item);
    nurbs_item->moveToGroup(item);
    ++i;
    }

  }

  dtkBRep* m_brep;
  Scene_cad_item* item;
};

Scene_cad_item::Scene_cad_item(dtkBRep* brep, CGAL::Three::Scene_interface* scene)
{
  d = new Scene_cad_item_priv(brep, scene, this);
}

void Scene_cad_item::computeElements()const
{

}

void Scene_cad_item::initializeBuffers(CGAL::Three::Viewer_interface *)
{
  CGAL::Three::Scene_group_item::initializeBuffers();
}
void Scene_cad_item::draw(CGAL::Three::Viewer_interface* viewer) const
{
  CGAL::Three::Scene_group_item::draw(viewer);
}

Scene_cad_item::~Scene_cad_item()
{
  if(d)
    delete d;
}

Scene_cad_item::Bbox Scene_cad_item::bbox() const
{
  Bbox box = Bbox(0,0,0,0,0,0);
  Q_FOREACH(Scene_item* item, getChildren())
  {
    box += item->bbox();
  }
 return box;
}
