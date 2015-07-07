#ifdef CGAL_GLEW_ENABLED
#include "GlSplat/GlSplat.h"
#endif


#include <CGAL/check_gl_error.h>
#include "config.h"
#include "Scene.h"
#include "Scene_item.h"

#include <QObject>
#include <QMetaObject>
#include <QString>
#include <QGLWidget>
#include <QEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QColorDialog>
#include <QApplication>
#include <QPointer>
#include <QList>
#include <QAbstractProxyModel>
#include <QDebug>
#include "Viewer_interface.h"

namespace {
void CGALglcolor(QColor c)
{
    //::glColor4d(c.red()/255.0, c.green()/255.0, c.blue()/255.0, c.alpha()/255.0);
}
}


//GlSplat::SplatRenderer* Scene::ms_splatting = 0;
//int Scene::ms_splattingCounter = 0;
//GlSplat::SplatRenderer* Scene::splatting()
//{
//    assert(ms_splatting!=0 && "A Scene object must be created before requesting the splatting object");
//    return ms_splatting;
//}


Scene::Scene(QObject* parent)
    : QAbstractListModel(parent),
      selected_item(-1),
      item_A(-1),
      item_B(-1)
{

    connect(this, SIGNAL(selectionRay(double, double, double,
                                      double, double, double)),
            this, SLOT(setSelectionRay(double, double, double,
                                       double, double, double)));

  /*  if(ms_splatting==0)
        ms_splatting  = new GlSplat::SplatRenderer();
    ms_splattingCounter++;
    */

}
Scene::Item_id
Scene::addItem(Scene_item* item)
{

    Bbox bbox_before = bbox();
    m_entries.push_back(item);
    connect(item, SIGNAL(itemChanged()),
            this, SLOT(itemChanged()));
    if(bbox_before + item->bbox() != bbox_before)
    {
        emit updated_bbox();
    }
#if QT_VERSION >= 0x050000
    QAbstractListModel::beginResetModel();
    emit updated();
    QAbstractListModel::endResetModel();
#else
    emit updated();
    QAbstractListModel::reset();
#endif
    Item_id id = m_entries.size() - 1;
    emit newItem(id);
    return id;
}

Scene_item*
Scene::replaceItem(Scene::Item_id index, Scene_item* item, bool emit_item_about_to_be_destroyed)
{
    item->changed();
    if(index < 0 || index >= m_entries.size())
        return 0;

    if(emit_item_about_to_be_destroyed) {
        emit itemAboutToBeDestroyed(m_entries[index]);
    }

    connect(item, SIGNAL(itemChanged()),
            this, SLOT(itemChanged()));
    std::swap(m_entries[index], item);
    if ( item->isFinite() && !item->isEmpty() &&
         m_entries[index]->isFinite() && !m_entries[index]->isEmpty() &&
         item->bbox()!=m_entries[index]->bbox() )
    {
        emit updated_bbox();
    }
    emit updated();
    itemChanged(index);
    // QAbstractListModel::reset();
    return item;
}

int
Scene::erase(int index)
{
    if(index < 0 || index >= m_entries.size())
        return -1;

    Scene_item* item = m_entries[index];
    emit itemAboutToBeDestroyed(item);
    delete item;
    m_entries.removeAt(index);

  selected_item = -1;

 #if QT_VERSION >= 0x050000
  QAbstractListModel::beginResetModel();
  emit updated();
  QAbstractListModel::endResetModel();
 #else
  emit updated();
  QAbstractListModel::reset();
 #endif


    if(--index >= 0)
        return index;
    if(!m_entries.isEmpty())
        return 0;
    return -1;
}

int
Scene::erase(QList<int> indices)
{
    QList<Scene_item*> to_be_removed;

    int max_index = -1;
    Q_FOREACH(int index, indices) {
        if(index < 0 || index >= m_entries.size())
            continue;
        max_index = (std::max)(max_index, index);
        Scene_item* item = m_entries[index];
        to_be_removed.push_back(item);
        emit itemAboutToBeDestroyed(item);
        delete item;
    }



  Q_FOREACH(Scene_item* item, to_be_removed) {
    m_entries.removeAll(item);
  }

  selected_item = -1;
 #if QT_VERSION >= 0x050000
  QAbstractListModel::beginResetModel();
  emit updated();
  QAbstractListModel::endResetModel();
 #else
  emit updated();
  QAbstractListModel::reset();
 #endif

  int index = max_index + 1 - indices.size();
  if(index >= m_entries.size()) {
    index = m_entries.size() - 1;
  }
  if(index >= 0)
    return index;
  if(!m_entries.isEmpty())
    return 0;
  return -1;

}

Scene::~Scene()
{
    Q_FOREACH(Scene_item* item_ptr, m_entries)
    {
        delete item_ptr;
    }
    m_entries.clear();

   // if((--ms_splattingCounter)==0)
   //     delete ms_splatting;
}

Scene_item*
Scene::item(Item_id index) const
{
    return m_entries.value(index); // QList::value checks bounds
}

Scene::Item_id 
Scene::item_id(Scene_item* scene_item) const
{
    return m_entries.indexOf(scene_item);
}

int
Scene::numberOfEntries() const
{
    return m_entries.size();
}

// Duplicate a scene item.
// Return the ID of the new item (-1 on error).
Scene::Item_id
Scene::duplicate(Item_id index)
{
    if(index < 0 || index >= m_entries.size())
        return -1;

    const Scene_item* item = m_entries[index];
    Scene_item* new_item = item->clone();
    if(new_item) {
        new_item->setName(tr("%1 (copy)").arg(item->name()));
        new_item->setColor(item->color());
        new_item->setVisible(item->visible());
        addItem(new_item);
        return m_entries.size() - 1;
    }
    else
        return -1;
}

void Scene::initializeGL()
{
  //  ms_splatting->init();

    //Setting the light options

    // Create light components
    GLfloat ambientLight[] = { 0.4f, 0.4f, 0.4f, 1.0f };
    GLfloat diffuseLight[] = { 1.0f, 1.0f, 1.0, 1.0f };
    GLfloat specularLight[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    GLfloat position[] = { 0.0f, 0.0f, 1.0f, 1.0f };

    // Assign created components to GL_LIGHT0
  //  glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
  //  glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
  //  glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight);
  //  glLightfv(GL_LIGHT0, GL_POSITION, position);

}

// workaround for Qt-4.2.
#if QT_VERSION < 0x040300
#  define lighter light
#endif

bool 
Scene::keyPressEvent(QKeyEvent* e){
    bool res=false;
    for (QList<int>::iterator it=selected_items_list.begin(),endit=selected_items_list.end();
         it!=endit;++it)
    {
        Scene_item* item=m_entries[*it];
        res |= item->keyPressEvent(e);
    }
    return res;
}

void 
Scene::draw()
{
    draw_aux(0);
}
void
Scene::draw(Viewer_interface* viewer)
{
    draw_aux(viewer);
}
void 
Scene::drawWithNames()
{
    drawWithNames(0);
}
void
Scene::drawWithNames(Viewer_interface* viewer)
{
    QOpenGLFunctions gl;
    gl.initializeOpenGLFunctions();

    std::vector<shaders_info> original_shaders;
    QColor bgColor(viewer->backgroundColor());

    //draws the image in the fbo
    for(int index = 0; index < m_entries.size(); ++index)
    {
        Scene_item& item = *m_entries[index];
        //transforms the index in a corresponding unique RGB color for picking.
        int R = (index & 0x000000FF) >>  0;
        int G = (index & 0x0000FF00) >>  8;
        int B = (index & 0x00FF0000) >> 16;
        float r= R/255.0;
        float g = G/255.0;
        float b = B/255.0;
        //The fragmentertex source code
        QString picking_fragment_source(
                    "void main(void) { \n"
                    "gl_FragColor = vec4(");
        picking_fragment_source.append(QString::number(r)+","+QString::number(g)+","+QString::number(b)+",1.0); \n"
                                                                                                        "} \n"
                                                                                                        "\n");

        foreach(QOpenGLShaderProgram* program, item.shader_programs)
        {
            for(int j=0; j<(int)program->shaders().size(); j++)
            {
                if(program->shaders().at(j)->shaderType() == QOpenGLShader::Fragment)
                {
                    //copies the original shaders of each program
                    shaders_info c;
                    c.code = program->shaders().at(j)->sourceCode();
                    c.program_index = item.shader_programs.key(program);
                    c.shader_index = j;
                    c.item_index = index;
                    original_shaders.push_back(c);
                    //replace their fragment shaders so they display with the picking color
                    program->shaders().at(j)->compileSourceCode(picking_fragment_source);
                }
                program->link();
            }
        }
        viewer->setBackgroundColor(::Qt::white);

        if(item.visible())
        {
            item.draw(viewer);
        }
    }
    //determines the size of the buffer
    int deviceWidth = viewer->camera()->screenWidth();
    int deviceHeight = viewer->camera()->screenHeight();
    int rowLength = deviceWidth * 4; // data asked in RGBA,so 4 bytes.
    const static int dataLength = rowLength * deviceHeight;
    GLubyte* buffer = new GLubyte[dataLength];

    // Qt uses upper corner for its origin while GL uses the lower corner.
    glReadPixels(picking_target.x(), deviceHeight-1-picking_target.y(), 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
    std::vector<QOpenGLShaderProgram*> all_programs(0);
    //resets the originals programs
    for(int i=0; i<(int)original_shaders.size(); i++)
    {
        int entries_index = original_shaders[i].item_index;
        int program_index = original_shaders[i].program_index;
        int shader_index = original_shaders[i].shader_index;
        m_entries[entries_index]->shader_programs[program_index]->shaders().at(shader_index)->compileSourceCode(original_shaders[i].code);
        m_entries[entries_index]->shader_programs[program_index]->link();
        all_programs.push_back(m_entries[entries_index]->shader_programs[program_index]);

    }

    int ID = (buffer[0] + buffer[1] * 256 +buffer[2] * 256*256);
    //if the picked color is not white (background color)
    if(buffer[0]*buffer[1]*buffer[2] < 255*255*255)
        viewer->setSelectedName(ID);
    else
        viewer->setSelectedName(-1);
    viewer->setBackgroundColor(bgColor);
    list_programs = all_programs;
}

void 
Scene::draw_aux(Viewer_interface* viewer)
{    // Flat/Gouraud OpenGL drawing
    for(int index = 0; index < m_entries.size(); ++index)
    {
        Scene_item& item = *m_entries[index];
        if(item.visible())
        {
            if(item.renderingMode() == Flat || item.renderingMode() == FlatPlusEdges || item.renderingMode() == Gouraud)
            {
                //      ::glPointSize(2.f);
                ::glLineWidth(1.0f);
                if(index == selected_item)
                {
                    item.selection_changed(true);
                    CGALglcolor(item.color().lighter(120));
                }
                else

                {
                    item.selection_changed(false);
                    CGALglcolor(item.color());
                }

                item.contextual_changed();
                if(viewer)
                    item.draw(viewer);
                else
                    item.draw();
            }
        }
    }


    // Wireframe OpenGL drawing
    for(int index = 0; index < m_entries.size(); ++index)
    {
        Scene_item& item = *m_entries[index];
        if(item.visible())
        {
            if(item.renderingMode() == FlatPlusEdges || item.renderingMode() == Wireframe)
            {
                //  ::glPointSize(2.f);
                ::glLineWidth(1.0f);
                if(index == selected_item)
                {
                    CGALglcolor(Qt::black);
                    item.selection_changed(true);
                }
                else
                {
                    CGALglcolor(item.color().lighter(50));
                    item.selection_changed(false);
                }

                item.contextual_changed();
                if(viewer)
                    item.draw_edges(viewer);
                else
                    item.draw_edges();
            }
            else{
                if( item.renderingMode() == PointsPlusNormals ){
                    //    ::glPointSize(2.f);
                    ::glLineWidth(1.0f);
                    if(index == selected_item)
                    {

                        item.selection_changed(true);
                        CGALglcolor(item.color().lighter(120));
                    }
                    else
                    {

                        item.selection_changed(false);
                        CGALglcolor(item.color());
                    }
                    item.contextual_changed();
                    if(viewer)
                        item.draw_edges(viewer);
                    else
                        item.draw_edges();
                }
            }
        }
    }

    // Points OpenGL drawing
    for(int index = 0; index < m_entries.size(); ++index)
    {
        Scene_item& item = *m_entries[index];
        if(item.visible())
        {
            if(item.renderingMode() == Points  || item.renderingMode() == PointsPlusNormals)
            {
                //   ::glPointSize(2.f);
                ::glLineWidth(1.0f);
                CGALglcolor(item.color());
                item.contextual_changed();
                if(viewer)
                    item.draw_points(viewer);
                else
                    item.draw_points();
            }
        }

    }

    // Splatting
   /* if(!with_names && ms_splatting->isSupported())
    {

        ms_splatting->beginVisibilityPass();
        for(int index = 0; index < m_entries.size(); ++index)
        {
            Scene_item& item = *m_entries[index];
            if(item.visible() && item.renderingMode() == Splatting)
            {

                if(viewer)
                {
                    item.draw_splats(viewer);
                }
                else
                    item.draw_splats();
            }

        }
       ms_splatting->beginAttributePass();
         for(int index = 0; index < m_entries.size(); ++index)
        {  Scene_item& item = *m_entries[index];
            if(item.visible() && item.renderingMode() == Splatting)
            {

                CGALglcolor(item.color());
                if(viewer)
                    item.draw_splats(viewer);
                else
                    item.draw_splats();
            }
        }
        ms_splatting->finalize();

    }*/
}

// workaround for Qt-4.2 (see above)
#undef lighter

int 
Scene::rowCount(const QModelIndex & parent) const
{
    if (parent.isValid())
        return 0;
    else
        return m_entries.size();
}

int 
Scene::columnCount(const QModelIndex & parent) const
{
    if (parent.isValid())
        return 0;
    else
        return NumberOfColumns;
}

QVariant 
Scene::data(const QModelIndex &index, int role) const
{
    if (!index.isValid())
        return QVariant();

    if(index.row() < 0 || index.row() >= m_entries.size())
        return QVariant();

    if(role == ::Qt::ToolTipRole)
    {
        return m_entries[index.row()]->toolTip();
    }
    switch(index.column())
    {
    case ColorColumn:
        if(role == ::Qt::DisplayRole || role == ::Qt::EditRole)
            return m_entries.value(index.row())->color();
        else if(role == ::Qt::DecorationRole)
            return m_entries.value(index.row())->color();
        break;
    case NameColumn:
        if(role == ::Qt::DisplayRole || role == ::Qt::EditRole)
            return m_entries.value(index.row())->name();
        if(role == ::Qt::FontRole)
            return m_entries.value(index.row())->font();
        break;
    case RenderingModeColumn:
        if(role == ::Qt::DisplayRole) {
            return m_entries.value(index.row())->renderingModeName();
        }
        else if(role == ::Qt::EditRole) {
            return static_cast<int>(m_entries.value(index.row())->renderingMode());
        }
        else if(role == ::Qt::TextAlignmentRole) {
            return ::Qt::AlignCenter;
        }
        break;
    case ABColumn:
        if(role == ::Qt::DisplayRole) {
            if(index.row() == item_A)
                return "A";
            if(index.row() == item_B)
                return "B";
        }
        else if(role == ::Qt::TextAlignmentRole) {
            return ::Qt::AlignCenter;
        }
        break;
    case VisibleColumn:
        if(role == ::Qt::DisplayRole || role == ::Qt::EditRole)
            return m_entries.value(index.row())->visible();
        break;
    default:
        return QVariant();
    }
    return QVariant();
}

QVariant 
Scene::headerData ( int section, ::Qt::Orientation orientation, int role ) const
{
    if(orientation == ::Qt::Horizontal)  {
        if (role == ::Qt::DisplayRole)
        {
            switch(section)
            {
            case NameColumn:
                return tr("Name");
                break;
            case ColorColumn:
                return tr("Color");
                break;
            case RenderingModeColumn:
                return tr("Mode");
            case ABColumn:
                return tr("A/B");
                break;
            case VisibleColumn:
                return tr("View");
                break;
            default:
                return QVariant();
            }
        }
        else if(role == ::Qt::ToolTipRole) {
            if(section == RenderingModeColumn) {
                return tr("Rendering mode (points/wireframe/flat/flat+edges/Gouraud)");
            }
            else if(section == ABColumn) {
                return tr("Selection A/Selection B");
            }
        }
    }
    return QAbstractListModel::headerData(section, orientation, role);
}

Qt::ItemFlags 
Scene::flags ( const QModelIndex & index ) const
{
    if (index.isValid() && index.column() == NameColumn) {
        return QAbstractListModel::flags(index) | ::Qt::ItemIsEditable;
    }
    else {
        return QAbstractListModel::flags(index);
    }
}

bool 
Scene::setData(const QModelIndex &index, 
               const QVariant &value,
               int role)
{
    if( role != ::Qt::EditRole || !index.isValid() )
        return false;

    if(index.row() < 0 || index.row() >= m_entries.size())
        return false;

    Scene_item* item = m_entries[index.row()];
    if(!item) return false;
    switch(index.column())
    {
    case NameColumn:
        item->setName(value.toString());
        item->changed();
        emit dataChanged(index, index);
        return true;
        break;
    case ColorColumn:
        item->setColor(value.value<QColor>());
        item->changed();
        emit dataChanged(index, index);
        return true;
        break;
    case RenderingModeColumn:
    {
        RenderingMode rendering_mode = static_cast<RenderingMode>(value.toInt());
        // Find next supported rendering mode
        while ( ! item->supportsRenderingMode(rendering_mode)
      //          || (rendering_mode==Splatting && !Scene::splatting()->isSupported())
                )
        {
            rendering_mode = static_cast<RenderingMode>( (rendering_mode+1) % NumberOfRenderingMode );
        }
        item->setRenderingMode(rendering_mode);
        item->changed();
        emit dataChanged(index, index);
        return true;
        break;
    }
    case VisibleColumn:
        item->setVisible(value.toBool());
        item->changed();
        emit dataChanged(index, index);
        return true;
    default:
        return false;
    }
    return false;
}

Scene::Item_id Scene::mainSelectionIndex() const {
    return selected_item;
}

QList<int> Scene::selectionIndices() const {
    return selected_items_list;
}

int Scene::selectionAindex() const {
    return item_A;
}

int Scene::selectionBindex() const {
    return item_B;
}

QItemSelection Scene::createSelection(int i)
{
    return QItemSelection(this->createIndex(i, 0),
                          this->createIndex(i, LastColumn));
}

QItemSelection Scene::createSelectionAll()
{
    return QItemSelection(this->createIndex(0, 0),
                          this->createIndex(m_entries.size() - 1 , LastColumn));
}

void Scene::itemChanged()
{
    Scene_item* item = qobject_cast<Scene_item*>(sender());
    if(item)
        itemChanged(item);
}

void Scene::itemChanged(Item_id i)
{
    if(i < 0 || i >= m_entries.size())
        return;

    m_entries[i]->changed();
    emit dataChanged(this->createIndex(i, 0),
                     this->createIndex(i, LastColumn));
}

void Scene::itemChanged(Scene_item* item)
{
    item->changed();
    emit dataChanged(this->createIndex(0, 0),
                     this->createIndex(m_entries.size() - 1, LastColumn));
}

bool SceneDelegate::editorEvent(QEvent *event, QAbstractItemModel *model,
                                const QStyleOptionViewItem &option,
                                const QModelIndex &index)
{
    QAbstractProxyModel* proxyModel = dynamic_cast<QAbstractProxyModel*>(model);
    Q_ASSERT(proxyModel);
    Scene *scene = dynamic_cast<Scene*>(proxyModel->sourceModel());
    Q_ASSERT(scene);
    switch(index.column()) {
    case Scene::VisibleColumn:
        if (event->type() == QEvent::MouseButtonPress) {
            QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
            if(mouseEvent->button() == ::Qt::LeftButton) {
                int x = mouseEvent->pos().x() - option.rect.x();
                if(x >= (option.rect.width() - size)/2 &&
                        x <= (option.rect.width() + size)/2) {
                    model->setData(index, ! model->data(index).toBool() );
                }
            }
            return false; //so that the selection can change
        }
        return true;
        break;
    case Scene::ColorColumn:
        if (event->type() == QEvent::MouseButtonPress) {
            QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
            if(mouseEvent->button() == ::Qt::LeftButton) {
                QColor color =
                        QColorDialog::getColor(model->data(index).value<QColor>(),
                                               0/*,
                                                                                                                                 tr("Select color"),
                                                                                                                                 QColorDialog::ShowAlphaChannel*/);
                if (color.isValid()) {
                    model->setData(index, color );
                }
            }
        }
        else if(event->type() == QEvent::MouseButtonDblClick) {
            return true; // block double-click
        }
        return false;
        break;
    case Scene::RenderingModeColumn:
        if (event->type() == QEvent::MouseButtonPress) {
            QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
            if(mouseEvent->button() == ::Qt::LeftButton) {
                // Switch rendering mode
                /*RenderingMode*/int rendering_mode = model->data(index, ::Qt::EditRole).toInt();
                rendering_mode = (rendering_mode+1) % NumberOfRenderingMode;
                model->setData(index, rendering_mode);
            }
        }
        else if(event->type() == QEvent::MouseButtonDblClick) {
            return true; // block double-click
        }
        return false;
        break;
    case Scene::ABColumn:
        if (event->type() == QEvent::MouseButtonPress) {
            if(index.row() == scene->item_B) {
                scene->item_A = index.row();
                scene->item_B = -1;
            }
            else if(index.row() == scene->item_A) {
                scene->item_B = index.row();
                scene->item_A = -1;
            }
            else if(scene->item_A == -1) {
                scene->item_A = index.row();
            }
            else {
                scene->item_B = index.row();
            }
            scene->dataChanged(scene->createIndex(0, Scene::ABColumn),
                               scene->createIndex(scene->rowCount() - 1, Scene::ABColumn));
        }
        return false;
        break;
    default:
        return QItemDelegate::editorEvent(event, model, option, index);
    }
}

void SceneDelegate::paint(QPainter *painter, const QStyleOptionViewItem &option,
                          const QModelIndex &index) const
{
    if (index.column() != Scene::VisibleColumn) {
        QItemDelegate::paint(painter, option, index);
    } else {
        const QAbstractItemModel *model = index.model();
        QPalette::ColorGroup cg = (option.state & QStyle::State_Enabled) ?
                    (option.state & QStyle::State_Active) ? QPalette::Normal : QPalette::Inactive : QPalette::Disabled;

        if (option.state & QStyle::State_Selected)
            painter->fillRect(option.rect, option.palette.color(cg, QPalette::Highlight));

        bool checked = model->data(index, ::Qt::DisplayRole).toBool();
        int width = option.rect.width();
        int height = option.rect.height();
        size = (std::min)(width, height);
        int x = option.rect.x() + (option.rect.width() / 2) - (size / 2);;
        int y = option.rect.y() + (option.rect.height() / 2) - (size / 2);
        if(checked) {
            painter->drawPixmap(x, y, checkOnPixmap.scaled(QSize(size, size),
                                                           ::Qt::KeepAspectRatio,
                                                           ::Qt::SmoothTransformation));
        }
        else {
            painter->drawPixmap(x, y, checkOffPixmap.scaled(QSize(size, size),
                                                            ::Qt::KeepAspectRatio,
                                                            ::Qt::SmoothTransformation));
        }
        drawFocus(painter, option, option.rect); // since we draw the grid ourselves
    }
}

void Scene::setItemVisible(int index, bool b)
{
    if( index < 0 || index >= m_entries.size() )
        return;
    m_entries[index]->setVisible(b);
    emit dataChanged(this->createIndex(index, VisibleColumn),
                     this->createIndex(index, VisibleColumn));
}

void Scene::setSelectionRay(double orig_x,
                            double orig_y,
                            double orig_z,
                            double dir_x,
                            double dir_y,
                            double dir_z)
{
    Scene_item* item = this->item(selected_item);
    if(item) item->select(orig_x,
                          orig_y,
                          orig_z,
                          dir_x,
                          dir_y,
                          dir_z);
}

void Scene::setItemA(int i)
{
    item_A = i;
    if(item_A == item_B)
    {
        item_B = -1;
    }
    emit dataChanged(this->createIndex(0, ABColumn),
                     this->createIndex(m_entries.size()-1, ABColumn));
}

void Scene::setItemB(int i)
{
    item_B = i;
    if(item_A == item_B)
    {
        item_A = -1;
    }
    emit updated();
    emit dataChanged(this->createIndex(0, ABColumn),
                     this->createIndex(m_entries.size()-1, ABColumn));
}

Scene::Bbox Scene::bbox() const
{
    if(m_entries.empty())
        return Bbox();

    bool bbox_initialized = false;
    Bbox bbox;
    Q_FOREACH(Scene_item* item, m_entries)
    {
        if(item->isFinite() && !item->isEmpty()) {
            if(bbox_initialized) {
                bbox = bbox + item->bbox();
            }
            else {
                bbox = item->bbox();
                bbox_initialized = true;
            }
        }
    }
    return bbox;
}

#include "Scene_find_items.h"

namespace scene { namespace details {

Q_DECL_EXPORT
Scene_item* 
findItem(const Scene_interface* scene_interface,
         const QMetaObject& metaobj,
         QString name, Scene_item_name_fn_ptr fn) {
    const Scene* scene = dynamic_cast<const Scene*>(scene_interface);
    if(!scene) return 0;
    Q_FOREACH(Scene_item* item, scene->entries()) {
        Scene_item* ptr = qobject_cast<Scene_item*>(metaobj.cast(item));
        if(ptr && ((ptr->*fn)() == name)) return ptr;
    }
    return 0;
}

Q_DECL_EXPORT
QList<Scene_item*> 
findItems(const Scene_interface* scene_interface, 
          const QMetaObject&,
          QString name, Scene_item_name_fn_ptr fn)
{
    const Scene* scene = dynamic_cast<const Scene*>(scene_interface);
    QList<Scene_item*> list;
    if(!scene) return list;

    Q_FOREACH(Scene_item* item, scene->entries()) {
        Scene_item* ptr = qobject_cast<Scene_item*>(item);
        if(ptr && ((ptr->*fn)() == name)) {
            list << ptr;
        }
    }
    return list;
}

} // end namespace details
                } // end namespace scene

#include "Scene.moc"
