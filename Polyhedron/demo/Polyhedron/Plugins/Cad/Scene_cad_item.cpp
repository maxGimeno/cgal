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
#include <CGAL/Three/Viewer_interface.h>
#include <QGLViewer/qglviewer.h>

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
    std::pair<int, int> pair;
    pair.first = 0;
    pair.second = 0;
    for(std::size_t id = 0; id< features.size(); ++id)
    {
      dtkNurbsCurve* nurb = features[id];
      std::size_t length = nurb->nbCps() + nurb->degree() - 1;
      double knots[length];
      nurb->knots(knots);
      dtkContinuousGeometryPrimitives::Point_3 p(0,0,0);
      nurb->evaluatePoint(knots[0], p.data());
      for(int j=0; j<3; ++j){
        intersection.push_back(p[j]);
        ++pair.second;
      }

      for(float f = knots[0]+1/100.0*(knots[length-1]-knots[0]);
          f<knots[length-1] - 1/100.0*(knots[length-1]-knots[0]);
          f+=1/100.0*(knots[length-1]-knots[0]))
      {
        nurb->evaluatePoint(f, p.data());
        for(int j=0; j<3; ++j){
          intersection.push_back(p[j]);
          ++pair.second;
        }
        for(int j=0; j<3; ++j){
          intersection.push_back(p[j]);
          ++pair.second;
        }
      }

      nurb->evaluatePoint(knots[length-1], p.data());
      for(int j=0; j<3; ++j){
        intersection.push_back(p[j]);
        ++pair.second;
      }
      trim_sizes.push_back(pair);
      pair.first = pair.second;
    }
    black_color.resize(intersection.size());
    for(int i=0; i<black_color.size(); ++i)
      black_color[i]=0.0f;
  }
  void initializeBuffers(CGAL::Three::Viewer_interface *viewer)const
  {
    // ///////////////////////////////////////////////////////////////////
    // Creates VBO EBO and VAO
    // ///////////////////////////////////////////////////////////////////
    item->buffers[B_COLOR].setUsagePattern(QOpenGLBuffer::DynamicCopy);
    m_program = item->getShaderProgram(Scene_nurbs_item::PROGRAM_NO_SELECTION, viewer);
    m_program->bind();
    item->vaos[INTERSECTION]->bind();
    item->buffers[B_INTERSECTION].bind();

    item->buffers[B_INTERSECTION].allocate(intersection.data(), static_cast<int>(intersection.size() * sizeof(float)));
    m_program->enableAttributeArray("vertex");
    m_program->setAttributeBuffer("vertex", GL_FLOAT, 0, 3, 0);
    item->buffers[B_INTERSECTION].release();

    item->buffers[B_COLOR].bind();
    item->buffers[B_COLOR].allocate(black_color.data(), static_cast<int>(black_color.size() * sizeof(float)));
    m_program->enableAttributeArray("colors");
    m_program->setAttributeBuffer("colors", GL_FLOAT, 0, 3, 0);
    item->buffers[B_INTERSECTION].release();
    item->vaos[INTERSECTION]->release();
    m_program->release();

    item->are_buffers_filled = true;
  }
  dtkBRep* m_brep;
  Scene_cad_item* item;
  mutable QOpenGLShaderProgram* m_program;
  mutable bool intersections_shown;
  mutable std::vector<float> intersection;
  mutable std::vector<float> black_color;
  mutable std::vector<int> trims_to_protect;
  mutable std::vector<std::pair<int, int> > trim_sizes;
  enum Vao
  {
    INTERSECTION=0,
    NumberOfVaos
  };

  enum Buffer
  {
    B_INTERSECTION=0,
    B_COLOR,
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
    actionShowTrimmed->setChecked(true);
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

void Scene_cad_item::checkTrimToProtect(int i)
{
  std::vector<int>::iterator it;
  for(it = d->trims_to_protect.begin();
      it != d->trims_to_protect.end();
      ++it)
  {
    if( *it == i)
    {
      break;
    }
  }
  if(it == d->trims_to_protect.end())
  {
    d->trims_to_protect.push_back(i);
  }
  else
  {
    d->trims_to_protect.erase(it);
  }
  CGAL::Three::Viewer_interface* viewer = static_cast<CGAL::Three::Viewer_interface*>(QGLViewer::QGLViewerPool().first());
  viewer->makeCurrent();
  d->m_program = getShaderProgram(PROGRAM_NO_SELECTION, viewer);
  d->m_program->bind();
  vaos[D::INTERSECTION]->bind();
  buffers[D::B_COLOR].bind();
  buffers[D::B_COLOR].write(0, d->black_color.data(), d->black_color.size()* sizeof(float));
  for(int i=0; i<d->trims_to_protect.size(); ++i)
  {
    int id = d->trims_to_protect[i];
    int size = d->trim_sizes[id].second - d->trim_sizes[id].first;
    std::vector<float> data(size);
    for(int j=0; j<size; j+=3)
    {
      data[j] = 1.0;
      data[j+1] = 0.85;
      data[j+2] = 0.15;
    }

    buffers[D::B_COLOR].write(d->trim_sizes[id].first*sizeof(float), data.data(), size * sizeof(float));
  }

  buffers[D::B_COLOR].release();
  vaos[D::INTERSECTION]->release();
  d->m_program->release();
  itemChanged();
}

const std::vector<int>& Scene_cad_item::trimsToProtect()const
{
  return d->trims_to_protect;
}

void Scene_cad_item::clearHighlight()
{
  d->trims_to_protect.clear();
  CGAL::Three::Viewer_interface* viewer = static_cast<CGAL::Three::Viewer_interface*>(QGLViewer::QGLViewerPool().first());
  viewer->makeCurrent();
  d->m_program = getShaderProgram(PROGRAM_NO_SELECTION, viewer);
  d->m_program->bind();
  vaos[D::INTERSECTION]->bind();
  buffers[D::B_COLOR].bind();
  buffers[D::B_COLOR].write(0, d->black_color.data(), d->black_color.size()* sizeof(float));
  for(int i=0; i<d->trims_to_protect.size(); ++i)
  {
    int id = d->trims_to_protect[i];
    int size = d->trim_sizes[id].second - d->trim_sizes[id].first;
    std::vector<float> data(size);
    for(int j=0; j<size; j+=3)
    {
      data[j] = 1.0;
      data[j+1] = 0.85;
      data[j+2] = 0.15;
    }

    buffers[D::B_COLOR].write(d->trim_sizes[id].first*sizeof(float), data.data(), size * sizeof(float));
  }

  buffers[D::B_COLOR].release();
  vaos[D::INTERSECTION]->release();
  d->m_program->release();
  itemChanged();
}
