/*******************************************************************************
 * Copyright (c) 2014 Black Rook Software
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Lesser Public License v2.1
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 ******************************************************************************/
package com.blackrook.physics2d;

/**
 * The listener object for collision events.
 * @author Matthew Tropiano
 */
public interface Physics2DListener<T extends CollisionBody2D>
{
	/**
	 * Called when a collision needs to be handled.
	 * The object passed into this is disposable.
	 * @param collision the collision event.
	 */
	public void handleCollision(Collision2D<T> collision);
}
