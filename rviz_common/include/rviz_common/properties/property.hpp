// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2017, Open Source Robotics Foundation, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#ifndef RVIZ_COMMON__PROPERTIES__PROPERTY_HPP_
#define RVIZ_COMMON__PROPERTIES__PROPERTY_HPP_

#include <string>

#include <QIcon>  // NOLINT: cpplint is unable to handle the include order here
#include <QObject>  // NOLINT: cpplint is unable to handle the include order here
#include <QString>  // NOLINT: cpplint is unable to handle the include order here
#include <QVariant>  // NOLINT: cpplint is unable to handle the include order here

#include "rviz_common/config.hpp"
#include "rviz_common/visibility_control.hpp"

class QPainter;
class QStyleOptionViewItem;

namespace rviz_common
{
namespace properties
{

class PropertyTreeModel;

/// A single element of a property tree, with a name, value, description, and possibly children.
/**
 * A Property in a property tree is a piece of data editable or at
 * least displayable in a PropertyTreeWidget.
 * A Property object owns the data item in question.
 * When client code needs to be informed about changes, it can connect to the
 * Property's aboutToChange() and changed() signals.
 * A slot receiving the changed() signal should then ask the Property itself
 * for the new data.
 * For example:
 *
 *   RangeDisplay::RangeDisplay()
 *   {
 *     color_property_ = new ColorProperty(
 *       "Color", Qt::white, "Color to draw the range.",
 *       this, SLOT(updateColorAndAlpha())
 *     );
 *
 *     alpha_property_ = new FloatProperty(
 *       "Alpha", 0.5, "Amount of transparency to apply to the range.",
 *       this, SLOT(updateColorAndAlpha())
 *     );
 *   }
 *
 *   void RangeDisplay::updateColorAndAlpha()
 *   {
 *     Ogre::ColourValue oc = color_property_->getOgreColor();
 *     float alpha = alpha_property_->getFloat();
 *     for (size_t i = 0; i < cones_.size(); ++i) {
 *       cones_[i]->setColor(oc.r, oc.g, oc.b, alpha);
 *     }
 *     context_->queueRender();
 *   }
 *
 * Many subclasses of Property exist with specializations for storing
 * specific types of data.
 * Most of these ultimately use Property::setValue() to store the data in a
 * QVariant.
 *
 * It is important that knowledge of subclasses not be *required* to
 * get and set data, so that external code can access and change
 * properties without needing to `#include` and link against the code
 * in question.
 * For instance:
 *
 *   prop->subProp("Offset")->subProp("X")->setValue(3.14);
 *
 * Sets the X component of the VectorProperty called "Offset" which is
 * a child of `prop`.
 * This works without this code needing to know about the existence of
 * VectorProperty.
 *
 * To show a Property tree in a PropertyTreeWidget, wrap the root
 * Property in a PropertyTreeModel and call PropertyTreeWidget::setModel()
 * with it.
 */
class RVIZ_COMMON_PUBLIC Property : public QObject
{
  Q_OBJECT

public:
  /// Constructor.
  /**
   * All parameters to the constructor are optional and can all be set
   * after construction as well.
   *
   * If a parent is given, this constructor calls `parent->addChild(this)`
   * to add itself to the parent's list of children.
   *
   * If `changed_slot` is given and either `parent` or `receiver` is
   * also, then the changed() signal is connected via QObject::connect() to the
   * slot described by `changed_slot` on the parent or the receiver.
   * If both parent and receiver are specified, receiver is the one which gets
   * connected.
   * If receiver is not specified, parent is used instead.
   *
   * \param name The name of this property.
   *   Appears in the left column of a PropertyTreeWidget.
   * \param default_value The initial value to store in the property.
   *   Appears in the right column of a PropertyTreeWidget.
   * \param description Text describing the property.
   *   Is shown in the "help" area of a PropertyTreeWithHelp widget.
   * \param parent The parent Property, or nullptr if there is no parent at
   *   this time.
   * \param changed_slot This should be a Qt slot specification,
   *   generated by Qt's @c SLOT() macro.
   *   It should be a slot on the `receiver` object, or if `receiver` is not
   *   specified, it should be a slot on the `parent`.
   * \param receiver If receiver is non-nullptr, the changed() signal is
   *   connected to the `changed_slot` on the `receiver` object.
   */
  explicit Property(
    const QString & name = QString(),
    const QVariant default_value = QVariant(),
    const QString & description = QString(),
    Property * parent = 0,
    const char * changed_slot = 0,
    QObject * receiver = 0);

  /// Destructor which also removes this property from its parent's list of children.
  virtual ~Property();

  /// Remove and delete some or all child Properties.
  /**
   * Does not change the value of this Property.
   *
   * Does not use numChildren() or takeChildAt(), operates directly on internal
   * children_ list.
   *
   * \param start_index The index of the first child to delete.
   * \param count The number of children to delete, or -1 to delete from
   *   start_index to the end of the list.
   */
  virtual
  void
  removeChildren(int start_index = 0, int count = -1);

  /// Set the new value for this property.
  /**
   * Returns true if the new value is different from the old value, false if
   * the same.
   *
   * If the new value is different from the old value, this emits
   * aboutToChange() before changing the value and emits changed() after.
   *
   * If the value set is an invalid QVariant (QVariant::isValid() returns
   * false), the value will not be editable in a PropertyTreeWidget.
   *
   * \param new_value The new value to store.
   * \return Returns true if new_value is different from current value,
   *   false if they are the same.
   */
  virtual
  bool
  setValue(const QVariant & new_value);

  /// Return the value of this Property as a QVariant.
  /**
   * If the value has never been set, an invalid QVariant is returned.
   */
  virtual
  QVariant
  getValue() const;

  /// Set the name.
  /**
   * Internally, the name is stored with QObject::setObjectName().
   *
   * \param name the new name.
   */
  virtual
  void
  setName(const QString & name);

  /// Return the name of this Property as a QString.
  virtual
  QString
  getName() const;

  /// Return the name of this Property as a std::string.
  std::string
  getNameStd() const;

  /// Set the description.
  /**
   * \param description the new description.
   */
  virtual
  void
  setDescription(const QString & description);

  /// Return the description.
  virtual
  QString
  getDescription() const;

  /// Set the icon to be displayed next to the property.
  virtual
  void
  setIcon(const QIcon & icon);

  /// Return the icon.
  virtual
  QIcon
  getIcon() const;

  /// Return the first child Property with the given name if found, else the FailureProperty.
  /**
   * If no child is found with the given name, an instance of a
   * special Property subclass named FailureProperty is returned and
   * an error message is printed to stdout.
   * FailureProperty::subProp() always returns itself, which means you
   * can safely chain a bunch of subProp() calls together and not have
   * a crash even if one of the sub-properties does not actually
   * exist.
   * For instance:
   *
   *   float width = prop->subProp("Dimenshons")->subProp("Width")->getValue().toFloat();
   *
   * If the first property `prop` has a "Dimensions" property but not
   * a "Dimenshons" one, `width` will end up set to `0` and an error
   * message will be printed, but the program will not crash here.
   *
   * This is an Order(N) operation in the number of subproperties.
   */
  virtual
  Property *
  subProp(const QString & sub_name);

  /// Return the number of child objects (Property or otherwise).
  /**
   * You can override this in a subclass to implement different child storage.
   */
  virtual
  int
  numChildren() const;

  /// Return the child Property with the given index, or nullptr.
  /**
   * nullptr is returned if the index is out of bounds or if the child at that
   * index is not a Property.
   *
   * This just checks the index against 0 and numChildren() and then
   * calls childAtUnchecked(), so it does not need to be overridden in
   * a subclass.
   */
  Property *
  childAt(int index) const;

  /// Return the child Property at the given index, without checking the bounds.
  /**
   * You can override this in a subclass to implement different child storage.
   */
  virtual
  Property *
  childAtUnchecked(int index) const;

  /// Return true if the list of children includes possible_child, false if not.
  bool
  contains(Property * possible_child) const;

  /// Return the parent Property.
  Property *
  getParent() const;

  /// Set parent property, without telling the parent.
  /**
   * Unlike specifying the parent property to the constructor,
   * setParent() does not have any immediate side effects, like adding
   * itself to be a child of the parent.
   * It should only be used by implementations of addChild() and takeChild()
   * and such.
   */
  void
  setParent(Property * new_parent);

  /// Return data appropriate for the given column (0 or 1) and role for this Property.
  /**
   * When overriding to add new data (like a color for example), check
   * the role for the thing you know about, and if it matches, return
   * your data.
   * If it does not match, call the parent class version of this function and
   * return its result.
   *
   * Return values from this function or overridden versions of it are where
   * background and foreground colors, check-box checked-state values, text,
   * and fonts all come from.
   *
   * \param column 0 for left column, 1 for right column.
   * \param role is a Qt::ItemDataRole
   */
  virtual
  QVariant
  getViewData(int column, int role) const;

  /// Return item flags appropriate for the given column (0 or 1) for this Property.
  /**
   * \param column 0 for left column, 1 for right column.
   * \return The Qt::ItemFlags for the given column of this property,
   *   including Qt::ItemIsSelectable, Qt::ItemIsEditable, etc.
   */
  virtual
  Qt::ItemFlags
  getViewFlags(int column) const;

  /// Hook to provide custom painting of the value data (right-hand column) in a subclass.
  /**
   * \param painter The QPainter to use.
   * \param option A QStyleOptionViewItem with parameters of the paint job,
   *   like the rectangle, alignments, etc.
   * \return true if painting has been done, false if not.
   *   The default implementation always returns false.
   *
   * To implement a custom appearance of a Property value, override this
   * function to do the painting and return true.
   *
   * If this function returns false, a QStyledItemDelegate will do the painting.
   */
  virtual
  bool
  paint(QPainter * painter, const QStyleOptionViewItem & option) const;

  /// Create an editor widget to edit the value of this property.
  /**
   * If this function returns nullptr, a QStyledItemDelegate will make an
   * editor widget.
   *
   * The widget returned by createEditor() must have one `Q_PROPERTY`
   * with `USER` set to `true`.
   * The PropertyTreeDelegate finds it, sets it with the results of
   * PropertyTreeModel::data() after creation, and after editing is finished it
   * reads it and calls PropertyTreeModel::setData() with the contents.
   *
   * \param parent The QWidget to set as the parent of the returned QWidget.
   * \param option A QStyleOptionViewItem with parameters of the editor widget,
   *   like the rectangle, alignments, etc.
   * \return the newly-created editor widget.
   *   The default implementation creates a QSpinBox for integer values, a
   *   FloatEdit for float or double values, or a QLineEdit for anything else.
   */
  virtual
  QWidget *
  createEditor(QWidget * parent, const QStyleOptionViewItem & option);

  /// Returns true if this Property is an ancestor of possible_child.
  /**
   * Ancestor means the parent or parent of parent etc.
   */
  bool
  isAncestorOf(Property * possible_child) const;

  /// Remove a given child object and return a pointer to it.
  /**
   * This performs a linear search through all the children.
   *
   * This uses only virtual functions, numChildren(), childAtUnchecked(),
   * and takeChildAt(), so it does not need to be virtual itself.
   *
   * \return If child is contained here, it is returned, otherwise nullptr.
   */
  Property *
  takeChild(Property * child);

  /// Take a child out of the child list, but don't destroy it.
  /**
   * \return Returns the child property at the given index, or nullptr if the
   *   index is out of bounds.
   *
   * This notifies the model about the removal.
   */
  virtual
  Property *
  takeChildAt(int index);

  /// Add a child property.
  /**
   * \param child The child property to add.
   * \param index [optional] The index at which to add the child.
   *   If less than 0 or greater than the number of child properties, the
   *   child will be added at the end.
   */
  virtual
  void
  addChild(Property * child, int index = -1);

  /// Set the model managing this Property and all its child properties, recursively.
  void
  setModel(PropertyTreeModel * model);

  /// Return the model managing this Property and its childrent.
  PropertyTreeModel *
  getModel() const;

  /// Return the row number of this property within its parent, or -1 if it has no parent.
  /**
   * This checks child_indexes_valid_ in the parent Property, and if it is
   * false calls reindexChildren().
   * Then returns row_number_within_parent_ regardless.
   */
  int
  rowNumberInParent() const;

  /// Move the child at from_index to to_index.
  virtual
  void
  moveChild(int from_index, int to_index);

  /// Load the value of this property and/or its children from the given Config reference.
  virtual
  void
  load(const Config & config);

  /// Write the value of this property and/or its children into the given Config reference.
  virtual
  void
  save(Config config) const;

  /// Returns true if the property is not read-only AND has data worth saving.
  bool
  shouldBeSaved() const;

  /// If save is true and getReadOnly() is false, shouldBeSaved will return true, otherwise false.
  /**
   * Default is true.
   */
  void
  setShouldBeSaved(bool save);

  /// If true, the children of this property should set their ItemIsEnabled flag to false.
  virtual
  bool
  getDisableChildren();

  /// Hide this Property in any PropertyTreeWidgets.
  /**
   * This is a convenience function which calls setHidden(true).
   * \see show(), setHidden(), getHidden()
   */
  void
  hide();

  /// Show this Property in any PropertyTreeWidgets.
  /**
   * This is a convenience function which calls setHidden(false).
   * \see show(), setHidden(), getHidden()
   */
  void
  show();

  /// Hide or show this property in any PropertyTreeWidget viewing its parent.
  /**
   * The hidden/shown state is not saved or loaded, it is expected to
   * be managed by the owner of the property.
   */
  virtual
  void
  setHidden(bool hidden);

  /// Return the hidden/shown state.
  /**
   * True means hidden, false means visible.
   */
  virtual
  bool
  getHidden() const;

  /// Prevent or allow users to edit this property from a PropertyTreeWidget.
  /**
   * This only applies to user edits.
   * Calling setValue() will still change the value.
   *
   * This is not inherently recursive.
   * Parents which need this to propagate to their children must override this
   * to implement that.
   */
  virtual
  void
  setReadOnly(bool read_only);

  /// Return the read-only-ness of this property.
  /**
   * \see setReadOnly()
   */
  virtual
  bool
  getReadOnly();

  /// Return whether this property is expanded or collapsed.
  virtual
  bool
  isExpanded();

  /// Collapse (hide the children of) this Property.
  /**
   * \note Properties start out collapsed by default.
   *
   * \see expand()
   */
  virtual
  void
  collapse();

  /// Expand (show the children of) this Property.
  /**
   * \note Properties start out collapsed by default.
   *
   * This function only works if the property is already owned by a
   * PropertyTreeModel connected to a PropertyTreeWidget.
   * If this is called and the model is subsequently attached to a widget, it
   * will not have any effect.
   *
   * \see collapse()
   */
  virtual
  void
  expand();

Q_SIGNALS:
  /// Emitted by setValue() just before the value has changed.
  void
  aboutToChange();

  /// Emitted by setValue() just after the value has changed.
  void
  changed();

  /// Emitted after insertions and deletions of child Properties.
  void
  childListChanged(rviz_common::properties::Property * this_property);

protected:
  /// Load the value of this property specifically, not including children.
  /**
   * This handles value_ types of string, double/float, bool, and int.
   * If config is invalid, this does nothing.
   */
  void
  loadValue(const Config & config);

  /// This is the central property value.
  /**
   * If you set it directly in a subclass, do so with care because many things
   * depend on the aboutToChange() and changed() events emitted by
   * setValue().
   */
  QVariant value_;

  /// Pointer to the PropertyTreeModel managing this property tree.
  /**
   * Any time there is a data value or structural change to the
   * properties in this tree, and model_ is non-nullptr, it must be
   * notified of the change.
   * Functions to notify it of changes include
   * PropertyTreeModel::beginInsert(),
   * PropertyTreeModel::endInsert(), PropertyTreeModel::beginRemove(),
   * PropertyTreeModel::endRemove(), and PropertyTreeModel::emitDataChanged().
   * The Property class already does this for itself, but subclasses must be
   * aware of it if they override functions which modify the structure or
   * contents of the tree.
   */
  PropertyTreeModel * model_;

  /// True if row_number_within_parent_ of all children is valid, false if not.
  /**
   * Subclasses should set this false when they add, remove, or reorder children.
   */
  bool child_indexes_valid_;

  QIcon icon_;

private:
  /// Set row_number_within_parent_ correctly for every child.
  /**
   * Sets child_indexes_valid_ to true when done.
   */
  void
  reindexChildren();

  Property * parent_;
  QList<Property *> children_;
  QString description_;
  bool hidden_;

  /// The property returned by subProp() when the requested name is not found.
  static Property * failprop_;

  int row_number_within_parent_;
  bool is_read_only_;
  bool is_expanded_;
  bool save_;
};

}  // namespace properties
}  // namespace rviz_common

#endif  // RVIZ_COMMON__PROPERTIES__PROPERTY_HPP_
