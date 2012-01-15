/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2012 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
 */

#include "AATTaskFactory.hpp"
#include "Task/Tasks/OrderedTask.hpp"
#include "Util/Macros.hpp"

static gcc_constexpr_data AbstractTaskFactory::LegalPointType aat_start_types[] = {
  AbstractTaskFactory::START_LINE,
  AbstractTaskFactory::START_CYLINDER,
  AbstractTaskFactory::START_SECTOR,
  AbstractTaskFactory::START_BGA,
};

static gcc_constexpr_data AbstractTaskFactory::LegalPointType aat_im_types[] = {
  AbstractTaskFactory::AAT_CYLINDER,
  AbstractTaskFactory::AAT_SEGMENT,
  AbstractTaskFactory::AAT_ANNULAR_SECTOR,
};

static gcc_constexpr_data AbstractTaskFactory::LegalPointType aat_finish_types[] = {
  AbstractTaskFactory::FINISH_LINE,
  AbstractTaskFactory::FINISH_CYLINDER,
  AbstractTaskFactory::FINISH_SECTOR,
};

AATTaskFactory::AATTaskFactory(OrderedTask& _task, const TaskBehaviour &tb)
  :AbstractTaskFactory(_task, tb,
                       LegalPointConstArray(aat_start_types,
                                            ARRAY_SIZE(aat_start_types)),
                       LegalPointConstArray(aat_im_types,
                                            ARRAY_SIZE(aat_im_types)),
                       LegalPointConstArray(aat_finish_types,
                                            ARRAY_SIZE(aat_finish_types)))
{
}

void 
AATTaskFactory::UpdateOrderedTaskBehaviour(OrderedTaskBehaviour& to)
{
  to.task_scored = true;
  to.homogeneous_tps = false;
  to.is_closed = false;
  to.min_points = 2;
  to.max_points = 13;
  to.start_requires_arm = true;
}

AbstractTaskFactory::LegalPointType
AATTaskFactory::GetMutatedPointType(const OrderedTaskPoint &tp) const
{
  const LegalPointType oldtype = GetType(tp);
  LegalPointType newtype = oldtype;

  switch (oldtype) {

  case START_SECTOR:
  case START_LINE:
  case START_CYLINDER:
  case START_BGA:
    break;

  case KEYHOLE_SECTOR:
  case BGAFIXEDCOURSE_SECTOR:
  case BGAENHANCEDOPTION_SECTOR:
    newtype = AbstractTaskFactory::GetMutatedPointType(tp);
    break;

  case FINISH_SECTOR:
  case FINISH_LINE:
  case FINISH_CYLINDER:
    break;

  case FAI_SECTOR:
    newtype = AAT_CYLINDER;
    //ToDo: create a 90 degree symmetric AAT sector
    break;

  case AST_CYLINDER:
    newtype = AAT_CYLINDER;
    break;

  case AAT_SEGMENT:
  case AAT_CYLINDER:
  case AAT_ANNULAR_SECTOR:
    break;
  }

  return newtype;
}
