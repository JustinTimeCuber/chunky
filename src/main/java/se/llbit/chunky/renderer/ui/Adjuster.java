/* Copyright (c) 2013 Jesper Öqvist <jesper@llbit.se>
 *
 * This file is part of Chunky.
 *
 * Chunky is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Chunky is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with Chunky.  If not, see <http://www.gnu.org/licenses/>.
 */
package se.llbit.chunky.renderer.ui;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.GroupLayout;
import javax.swing.GroupLayout.Group;
import javax.swing.JLabel;
import javax.swing.JSlider;
import javax.swing.JTextField;
import javax.swing.LayoutStyle.ComponentPlacement;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

/**
 * Adjusts a rendering parameter.
 *
 * @author Jesper Öqvist <jesper@llbit.se>
 */
public abstract class Adjuster implements ChangeListener, ActionListener {
	private final JLabel lbl;
	private final JSlider slider;
	private final JTextField textField;
	private final double min;
	private final double max;
	private final boolean integerMode;

	/**
	 * Create new double value adjuster
	 * @param label
	 * @param tip
	 * @param min
	 * @param max
	 */
	public Adjuster(String label, String tip, double min, double max) {
		this.min = min;
		this.max = max;
		lbl = new JLabel(label);
		slider = new JSlider(1, 100);
		slider.setToolTipText(tip);
		slider.addChangeListener(this);
		textField = new JTextField(5);
		textField.addActionListener(this);
		integerMode = false;
	}

	/**
	 * Create new integer value adjuster
	 * @param label
	 * @param tip
	 * @param min
	 * @param max
	 */
	public Adjuster(String label, String tip, int min, int max) {
		this.min = min;
		this.max = max;
		lbl = new JLabel(label);
		slider = new JSlider(min, max);
		slider.setToolTipText(tip);
		slider.addChangeListener(this);
		textField = new JTextField(5);
		textField.addActionListener(this);
		integerMode = true;
	}

	/**
	 * @param layout
	 * @return horizontal layout group
	 */
	public Group horizontalGroup(GroupLayout layout) {
		return layout.createSequentialGroup()
				.addComponent(lbl)
				.addPreferredGap(ComponentPlacement.RELATED)
				.addComponent(slider)
				.addPreferredGap(ComponentPlacement.RELATED)
				.addComponent(textField);
	}

	/**
	 * @param layout
	 * @return vertical layout group
	 */
	public Group verticalGroup(GroupLayout layout) {
		return layout.createParallelGroup()
				.addComponent(lbl)
				.addComponent(slider)
				.addComponent(textField, GroupLayout.PREFERRED_SIZE, GroupLayout.PREFERRED_SIZE, GroupLayout.PREFERRED_SIZE);
	}

	@Override
	public void actionPerformed(ActionEvent e) {
		// TODO Auto-generated method stub

	}
	@Override
	public void stateChanged(ChangeEvent e) {
		JSlider source = (JSlider) e.getSource();
		double scale = (max - min) / (source.getMaximum() - source.getMinimum());
		double value = (source.getValue() - source.getMinimum()) * scale + min;
		setTextField(value);
		valueChanged(value);
	}

	/**
	 * Set the parameter value
	 * @param value
	 */
	public void set(double value) {
		slider.removeChangeListener(this);
		double scale = (slider.getMaximum() - slider.getMinimum()) / (max - min);
		int sliderValue = (int) ((value - min) * scale + 0.5 + slider.getMinimum());
		slider.setValue(sliderValue);
		slider.addChangeListener(this);
		setTextField(value);
	}

	private void setTextField(double value) {
		textField.removeActionListener(this);
		if (integerMode) {
			textField.setText("" + (int) value);
		} else {
			textField.setText("" + value);
		}
		textField.addActionListener(this);
	}

	/**
	 * Handles value changes
	 * @param newValue
	 */
	public abstract void valueChanged(double newValue);
}