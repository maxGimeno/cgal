#define foreach Q_FOREACH
#include <dtkBRep>
#include <dtkNurbsSurface>
#include <dtkTopoTrim>
#include <dtkNurbsCurve>
#include <dtkTrimLoop>
#include <dtkTrim>
#include <dtkContinuousGeometryUtils>

#include "Scene_cad_item.h"
#include "Scene_nurbs_item.h"
#include <QDebug>
#include <QMenu>

struct Scene_cad_item_priv{
  Scene_cad_item_priv(dtkBRep* brep, CGAL::Three::Scene_interface* scene, Scene_cad_item* parent)
    :m_brep(brep), item(parent), intersections_shown(false)
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

    std::set< const dtkTopoTrim *> topo_trims;
    std::vector<dtkNurbsCurve*> features;
    // ///////////////////////////////////////////////////////////////////
    // Iterates on all the trims, check if the topo_trim has been found, if it has, add the three curves as a tuple
    // Else add the topo trim and the first trim found attached to it
    // As the BRep model is a closed polysurface, for each topo trim there should be two trims associated to it
    // ///////////////////////////////////////////////////////////////////
    for (auto it = nurbs_surfaces.begin(); it != nurbs_surfaces.end(); ++it) {
      for (auto trim_loop = (*it)->trimLoops().begin(); trim_loop != (*it)->trimLoops().end(); ++trim_loop) {
        for (auto trim = (*trim_loop)->trims().begin(); trim != (*trim_loop)->trims().end(); ++trim)
        {
          if((*trim)->topoTrim()->m_nurbs_curve_3d != nullptr) {
            auto topo_trim = topo_trims.find((*trim)->topoTrim());
            if (topo_trim == topo_trims.end()) {
              topo_trims.insert((*trim)->topoTrim());
            } else {
              features.push_back((*trim)->topoTrim()->m_nurbs_curve_3d);
            }
          }
        }
      }
    }

    intersection.resize(0);

    for(std::size_t id = 0; id< features.size(); ++id)
    {
      dtkNurbsCurve* nurb = features[id];
      std::size_t length = nurb->nbCps() + nurb->degree() - 1;
      double knots[length];
      nurb->knots(knots);
      dtkContinuousGeometryPrimitives::Point_3 p(0,0,0);
      nurb->evaluatePoint(knots[0], p.data());
      for(int j=0; j<3; ++j)
        intersection.push_back(p[j]);

      for(float f = knots[0]+1/100.0*(knots[length-1]-knots[0]);
          f<knots[length-1] - 1/100.0*(knots[length-1]-knots[0]);
          f+=1/100.0*(knots[length-1]-knots[0]))
      {
        nurb->evaluatePoint(f, p.data());
        for(int j=0; j<3; ++j)
          intersection.push_back(p[j]);
        for(int j=0; j<3; ++j)
          intersection.push_back(p[j]);
      }

      nurb->evaluatePoint(knots[length-1], p.data());
      for(int j=0; j<3; ++j)
        intersection.push_back(p[j]);
    }
  }
  void initializeBuffers(CGAL::Three::Viewer_interface *viewer)const
  {
    // ///////////////////////////////////////////////////////////////////
    // Creates VBO EBO and VAO
    // ///////////////////////////////////////////////////////////////////
    m_program = item->getShaderProgram(Scene_nurbs_item::PROGRAM_NO_SELECTION, viewer);
    m_program->bind();
    item->vaos[INTERSECTION]->bind();
    item->buffers[B_INTERSECTION].bind();

    item->buffers[B_INTERSECTION].allocate(intersection.data(), static_cast<int>(intersection.size() * sizeof(float)));
    m_program->enableAttributeArray("vertex");
    m_program->setAttributeBuffer("vertex", GL_FLOAT, 0, 3, 0);
    m_program->disableAttributeArray("colors");
    item->buffers[B_INTERSECTION].release();
    item->vaos[INTERSECTION]->release();
    m_program->release();
  }
  dtkBRep* m_brep;
  Scene_cad_item* item;
  mutable QOpenGLShaderProgram* m_program;
  mutable bool intersections_shown;
  mutable std::vector<float> intersection;
  enum Vao
  {
    INTERSECTION=0,
    NumberOfVaos
  };

  enum Buffer
  {
    B_INTERSECTION=0,
    NumberOfBuffers
  };

};

typedef Scene_cad_item_priv D;
Scene_cad_item::Scene_cad_item(dtkBRep* brep, CGAL::Three::Scene_interface* scene)
  :CGAL::Three::Scene_group_item("unnamed", D::NumberOfBuffers, D::NumberOfVaos)
{
  d = new Scene_cad_item_priv(brep, scene, this);
}

void Scene_cad_item::computeElements()const
{

}


void Scene_cad_item::draw(CGAL::Three::Viewer_interface* viewer) const
{
  CGAL::Three::Scene_group_item::draw(viewer);
  if(d->intersections_shown)
  {
    if(!are_buffers_filled)
      d->initializeBuffers(viewer);

    attribBuffers(viewer, PROGRAM_NO_SELECTION);
    d->m_program = getShaderProgram(PROGRAM_NO_SELECTION, viewer);
    d->m_program->bind();
    vaos[D::INTERSECTION]->bind();
    d->m_program->setAttributeValue("colors", QColor(Qt::black));
    viewer->glDrawArrays(GL_LINES,0, d->intersection.size()/3);
    vaos[D::INTERSECTION]->release();
    d->m_program->release();
  }
}

void Scene_cad_item::drawEdges(CGAL::Three::Viewer_interface* viewer) const
{
  CGAL::Three::Scene_group_item::drawEdges(viewer);
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

void Scene_cad_item::show_trimmed(bool b)
{
  Q_FOREACH(Scene_item* item, getChildren())
  {
    Scene_nurbs_item* nurbs = qobject_cast<Scene_nurbs_item*>(item);
    if(!nurbs)
      continue;

    nurbs->show_trimmed(b);
  }
}

void Scene_cad_item::show_intersections(bool b)
{
  d->intersections_shown = b;
  itemChanged();
}

QMenu* Scene_cad_item::contextMenu()
{
  const char* prop_name = "Menu modified by Scene_cad_item.";

  QMenu* menu = Scene_item::contextMenu();

  // Use dynamic properties:
  // http://doc.qt.io/qt-5/qobject.html#property
  bool menuChanged = menu->property(prop_name).toBool();

  if (!menuChanged) {
    QAction* actionShowTrimmed=
        menu->addAction(tr("Show Trimmed"));
    actionShowTrimmed->setCheckable(true);
    actionShowTrimmed->setChecked(false);
    actionShowTrimmed->setObjectName("actionShowTrimmed");
    connect(actionShowTrimmed, SIGNAL(toggled(bool)),
            this, SLOT(show_trimmed(bool)));

    QAction* actionShowIntersections=
        menu->addAction(tr("Show Intersections"));
    actionShowIntersections->setCheckable(true);
    actionShowIntersections->setChecked(false);
    actionShowIntersections->setObjectName("actionShowIntersections");
    connect(actionShowIntersections, SIGNAL(toggled(bool)),
            this, SLOT(show_intersections(bool)));

    menu->setProperty(prop_name, true);
  }
  return menu;
}

dtkBRep* Scene_cad_item::brep()
{
  return d->m_brep;
}

const dtkBRep* Scene_cad_item::brep() const
{
  return d->m_brep;
}

QString Scene_cad_item::toolTip() const
{
  QString str =
         QObject::tr("<p>BRep <b>%1</b>")
            .arg(this->name());
  return str;
}
