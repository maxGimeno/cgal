//! \file Scene.h
#ifndef SCENE_H
#define SCENE_H
#include "config.h"
#include "Scene_config.h"
#include <CGAL/Three/Scene_interface.h>
#include <CGAL/Three/Scene_draw_interface.h>
#include <CGAL/Three/Viewer_config.h>

#include <QtOpenGL/qgl.h>
#include <QStandardItemModel>
#include <QString>
#include <QColor>
#include <QList>
#include <QMap>
#include <QItemDelegate>
#include <QPixmap>
#include <QItemSelection>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QGLViewer/qglviewer.h>
#include <QDebug>
#include <iostream>
#include <cmath>
#include <boost/variant.hpp>
#include <CGAL/Three/Scene_group_item.h>

class QEvent;
class QMouseEvent;
namespace CGAL { namespace Three{ class Viewer_interface;}}

//! This class is not supposed to be used by Plugins, but sometimes you may need access to
//! peculiar signals or slots.
class SCENE_EXPORT Scene  :
  public QStandardItemModel, public CGAL::Three::Scene_interface, public CGAL::Three::Scene_draw_interface

{
  Q_OBJECT
  Q_PROPERTY(int numberOfEntries READ numberOfEntries)

  friend class SceneDelegate;

public:
  QList<QModelIndex> getModelIndexFromId(int id) const;
  int getIdFromModelIndex(QModelIndex modelId) const;
  enum Columns { LockColumn = 0,
                 NameColumn ,
                 ColorColumn,
                 RenderingModeColumn,
                 VisibleColumn,
                 ABColumn,
                 LastColumn = ABColumn,
                 NumberOfColumns = LastColumn + 1};
  Scene(QObject*  parent);
  ~Scene();
  int addItem(CGAL::Three::Scene_item* item) Q_DECL_OVERRIDE;
  void changeGroup(CGAL::Three::Scene_item* item, CGAL::Three::Scene_group_item* target_group) Q_DECL_OVERRIDE;
  CGAL::Three::Scene_item* replaceItem(int index, CGAL::Three::Scene_item* item, bool emit_item_about_to_be_destroyed = false) Q_DECL_OVERRIDE;
  Q_INVOKABLE int erase(int) Q_DECL_OVERRIDE;

  int erase(QList<int>) Q_DECL_OVERRIDE;
  int duplicate(int index) Q_DECL_OVERRIDE;

  // Accessors (getters)
  int numberOfEntries() const Q_DECL_OVERRIDE;
  // returns the list of items.
  const QList<CGAL::Three::Scene_item*>& entries() const { return m_entries; }
  Q_INVOKABLE CGAL::Three::Scene_item* item(int) const Q_DECL_OVERRIDE;
  int item_id(CGAL::Three::Scene_item*) const Q_DECL_OVERRIDE;
  
  //! \todo Replace Index based selection functionality with those
  //! functions.
  ///@{
  CGAL::Three::Scene_item* selectedItem() const;
  QList<CGAL::Three::Scene_item*> selectedItems() const;
  QList<CGAL::Three::Scene_item*> selectionA() const;
  QList<CGAL::Three::Scene_item*> selectionB() const;
  ///@}

  int mainSelectionIndex() const Q_DECL_OVERRIDE;
  QList<int> selectionIndices() const Q_DECL_OVERRIDE;
  int selectionAindex() const Q_DECL_OVERRIDE;
  int selectionBindex() const Q_DECL_OVERRIDE;
  void initializeGL(CGAL::Three::Viewer_interface*) Q_DECL_OVERRIDE;
  void setPickedPixel(const QPoint &p) Q_DECL_OVERRIDE {picked_pixel = p;}
  void draw(CGAL::Three::Viewer_interface*) Q_DECL_OVERRIDE;
  void drawWithNames(CGAL::Three::Viewer_interface*) Q_DECL_OVERRIDE;
  bool keyPressEvent(QKeyEvent* e) Q_DECL_OVERRIDE;
  void printPrimitiveId(QPoint point,
                        CGAL::Three::Viewer_interface*) Q_DECL_OVERRIDE;
  void printVertexIds(CGAL::Three::Viewer_interface*) Q_DECL_OVERRIDE;
  void printEdgeIds(CGAL::Three::Viewer_interface*) Q_DECL_OVERRIDE;
  void printFaceIds(CGAL::Three::Viewer_interface*) Q_DECL_OVERRIDE;
  void printAllIds(CGAL::Three::Viewer_interface*) Q_DECL_OVERRIDE;
  //!Re-computes the primitiveIds for `item`
  void updatePrimitiveIds(Viewer_interface *, Scene_item *item) Q_DECL_OVERRIDE;
  bool testDisplayId(double x, double y, double z, CGAL::Three::Viewer_interface* viewer) Q_DECL_OVERRIDE;
  Bbox bbox(bool all = false) const Q_DECL_OVERRIDE;
  double len_diagonal() const Q_DECL_OVERRIDE
  {
    Bbox box = bbox();
    double dx = box.xmax() - box.xmin();
    double dy = box.ymax() - box.ymin();
    double dz = box.zmax() - box.zmin();
    return std::sqrt(dx*dx + dy*dy + dz*dz);
  }

  //Moves a name up in the Geometric Objects view
  void moveRowUp();
  //Moves a name down in the Geometric Objects view
  void moveRowDown();
  // QStandardItemModel functions
  //Defines the behavior when a name is drag-and-dropped in the Geometric Objects view
  bool dropMimeData(const QMimeData *, Qt::DropAction, int, int,
                    const QModelIndex &parent) Q_DECL_OVERRIDE;
  //Contains the text and icons of an item in the Geometric Objects view
  QVariant data ( const QModelIndex & index,
                  int role = ::Qt::DisplayRole ) const Q_DECL_OVERRIDE;
  //@returns the type of data correspondind to the role.
  QVariant headerData ( int section, ::Qt::Orientation orientation,
                        int role = ::Qt::DisplayRole ) const Q_DECL_OVERRIDE;
  //@returns the flags for the item at the target index.
  ::Qt::ItemFlags flags ( const QModelIndex & index ) const Q_DECL_OVERRIDE;
  // Sets the column data for the target index. Returns false if index is not valid and
  // if role is not EditRole.
  bool setData(const QModelIndex &index, const QVariant &value,
               int role) Q_DECL_OVERRIDE;

  //Returns a list of all the items.
  QList<CGAL::Three::Scene_item*> item_entries() const ;
  // auxiliary public function for QMainWindow
  //Selects the row at index i in the sceneView.
  QItemSelection createSelection(int i);
  //Selects all the rows in the sceneView.
  QItemSelection createSelectionAll();
  //Connects specific signals to a group when it is added and
  // gives a reference to the scene to it.
  void addGroup(Scene_group_item* group);

  void zoomToPosition(QPoint point,
                        CGAL::Three::Viewer_interface*) Q_DECL_OVERRIDE;

public Q_SLOTS:
  //!Specifies a group as Expanded for the Geometric Objects view
  void setExpanded(QModelIndex);
  //!Specifies a group as Collapsed for the Geometric Objects view
  void setCollapsed(QModelIndex);
  //!Transmits a CGAL::Three::Scene_item::itemChanged() signal to the scene.
  void itemChanged();
  void itemChanged(int i) Q_DECL_OVERRIDE;
  void itemChanged(CGAL::Three::Scene_item*) Q_DECL_OVERRIDE;
  //!Transmits a CGAL::Three::Scene_item::itemVisibilityChanged() signal to the scene.
  void itemVisibilityChanged();
  void itemVisibilityChanged(CGAL::Three::Scene_item*) Q_DECL_OVERRIDE;
  //!Removes `item` from all the groups of the scene.
  void remove_item_from_groups(CGAL::Three::Scene_item* item);
  //!Re-organizes the sceneView.
  void redraw_model();
  //! Sets the selected item to the target index. Emits a signal to notify
  //! that a new item is now selected, but does not update the Geometric Objects view.
  //! Used in intern and by the mainwindow
  void setSelectedItemIndex(int i)
  {
    selected_item = i;
    Q_EMIT itemIndexSelected(i);
  }
  void setSelectedItem(int i ) Q_DECL_OVERRIDE
  {
    selected_item = i;
    Q_EMIT selectionChanged(i);
  }

  //! Does the same as setSelectedItem(int)
  void setSelectedItem(CGAL::Three::Scene_item* item_to_select)
  {
    int i=0;
    Q_FOREACH(CGAL::Three::Scene_item* item, m_entries)
    {
      if (item==item_to_select)
      {
        setSelectedItem(i);
        break;
      }
      ++i;
    }
  }
  //! Sets the target list of indices as the selected indices.
  QList<int> setSelectedItemsList(QList<int> l )
  {
    Q_FOREACH(int i,l)
    {
       CGAL::Three::Scene_group_item* group =
               qobject_cast<CGAL::Three::Scene_group_item*>(item(i));
       if(group)
       {
         QList<int> list;
         Q_FOREACH(Item_id id, group->getChildrenIds())
           list<<id;
         l << setSelectedItemsList(list);
       }

    }
    selected_items_list = l;
    return l;
  }

  // Accessors (setters)
  //!Sets the item at index i to visible or not visible.
  void setItemVisible(int, bool b);
  //!Sets the item_A as the item at index i .
  void setItemA(int i);
  //!Sets the item_B as the item at index i .
  void setItemB(int i);

Q_SIGNALS:
  //generated automatically by moc
  //! Emit this to mark `modelindex` as selected in the Geometric Objects view.
  void itemPicked(const QModelIndex& modelindex);
  //! Is emitted when a new item is added to the scene.
  void newItem(int);
  //! Emit this to re-compute the viewer's Bbox;
  //! If `b` is true, the scene will be recentered
  void updated_bbox(bool b);
  //! Emit this to redraw the scene.
  void updated();
  //! Is emitted when `item` is erased.
  void itemAboutToBeDestroyed(CGAL::Three::Scene_item* item);
  //!Is emitted when the ids of the items are changed.
  void indexErased(Scene_interface::Item_id id);
  //! Is emitted when the selection ray is changed.
  void selectionRay(double, double, double, double, double, double);
  //! Used to update the selected item in the Geometric Objects view.
  void selectionChanged(int i);
  //! Used when you don't want to update the selectedItem in the Geometric Objects view.
  void itemIndexSelected(int i);
  //! Emit this to reset the collapsed state of all groups after the Geometric Objects view has been redrawn.
  void restoreCollapsedState();
  //! Is emitted when draw() is finished.
  void drawFinished();
private Q_SLOTS:
  // Casts a selection ray and calls the item function select.
  void setSelectionRay(double, double, double, double, double, double);
  void s_itemAboutToBeDestroyed(CGAL::Three::Scene_item *);
  void adjustIds(Scene_interface::Item_id removed_id);
  void callDraw(){
    Q_FOREACH(QGLViewer* viewer, QGLViewer::QGLViewerPool())
    {
      if(viewer == NULL)
        continue;
      viewer->update();
    }
  }
private:
  /*! Calls the drawing functions of each visible item according
   * to its current renderingMode. If with_names is true, uses
   * the OpenGL mode GL_WITH_NAMES, essentially used for the picking.*/
  void draw_aux(bool with_names, CGAL::Three::Viewer_interface*);
  bool has_alpha();
  void renderScene(const QList<Scene_interface::Item_id > &items,
                   CGAL::Three::Viewer_interface* viewer, QMap<float, int> &picked_item_IDs, bool with_names,
                   int pass, bool writing_depth,
                   QOpenGLFramebufferObject* fbo);
  void renderWireScene(const QList<Scene_interface::Item_id> &items,
                       Viewer_interface *viewer, QMap<float, int> &picked_item_IDs, bool with_names);
  void renderPointScene(const QList<Scene_interface::Item_id> &items,
                        Viewer_interface *viewer,
                        QMap<float, int>& picked_item_IDs,
                        bool with_names);

  // Re-draw the hierarchy of the view.
  void organize_items(CGAL::Three::Scene_item* item, QStandardItem *root, int loop);
  // List of Scene_items.
  typedef QList<CGAL::Three::Scene_item*> Entries;
  //List containing all the scene_items.
  Entries m_entries;
  //List containing the indices of the items without parents.
  QList<Item_id> children;
  // Index of the currently selected item.
  int selected_item;
  //List of indices of the currently selected items.
  QList<int> selected_items_list;
  //Index of the item_A.
  int item_A;
  //Index of the item_B.
  int item_B;
  QPoint picked_pixel;
  bool gl_init;
  QMap<QModelIndex, int> index_map;
  QOpenGLShaderProgram program;
  QMap<CGAL::Three::Viewer_interface*, QOpenGLVertexArrayObject*> vaos;
  QOpenGLBuffer vbo[2];
  float points[18];
  float uvs[12];
  void initGL(CGAL::Three::Viewer_interface* viewer);

public:
  void newViewer(CGAL::Three::Viewer_interface*);
  void removeViewer(CGAL::Three::Viewer_interface*);

}; // end class Scene

class QAbstractProxyModel;

//\brief The SceneDelegate class
//Handles the columns of the sceneView
class SCENE_EXPORT SceneDelegate : public QItemDelegate
{
public:
  SceneDelegate(QObject * parent = 0)
    : QItemDelegate(parent),
      checkOnPixmap(":/cgal/icons/check-on.png"),
      checkOffPixmap(":/cgal/icons/check-off.png"),
      readingPixmap(":/cgal/icons/resources/reading_pic.png"),
      writingPixmap(":/cgal/icons/resources/writing_pic.png")
  {
  }
// Handles the clicks on the sceneView
  bool editorEvent(QEvent *event, QAbstractItemModel *model,
                   const QStyleOptionViewItem &option,
                   const QModelIndex &index);
  // Draws the content of the sceneView
  void paint(QPainter *painter, const QStyleOptionViewItem &option,
             const QModelIndex &index) const;
  void setProxy(QAbstractProxyModel* p_proxy){
      proxy = p_proxy;
  }
  void setScene(Scene* p_scene){
      scene = p_scene;
  }

private:
  QPixmap checkOnPixmap;
  QPixmap checkOffPixmap;
  QPixmap readingPixmap;
  QPixmap writingPixmap;
  QAbstractProxyModel *proxy;
  Scene *scene;
  mutable int size;
}; // end class SceneDelegate

#endif // SCENE_H


