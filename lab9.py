import numpy as np  # Импортирование библиотеки numpy
import unittest  # Импортирование библиотеки unittest (для тестов)


def calculate_diagonal_sum(matrix):  # Функция для вычисления суммы чисел главной диагонали матрицы
    return np.trace(matrix)  # Возвращает сумму чисел главной диагонали матрицы


class TestDiagonalSum(unittest.TestCase):  # Тестирование функции, вычисляющей сумму чисел главной диагонали

    def test_square_matrix(self):  # Квадратная матрица
        matrix = np.array([[1, 1], [1, 2]])  # Создаем квадратную матрицу размера 2x2
        self.assertEqual(calculate_diagonal_sum(matrix), 3)  # Проверяем ответ

    def test_identity_matrix(self):  # Матрица из единиц
        matrix = np.array([[1, 1, 1], [1, 1, 1], [1, 1, 1]])  # 3x3 единичная матрица
        self.assertEqual(calculate_diagonal_sum(matrix), 3)  # Проверяем ответ

    def test_zero_matrix(self):  # Тестирование функции на матрице из нулей
        matrix = np.zeros((2, 2))  # 2x2 нулевая матрица
        self.assertEqual(calculate_diagonal_sum(matrix), 0)  # Проверяем ответ

    def test_negative_elements(self):  # Тестирование функции на матрице с отрицательными элементами
        matrix = np.array([[1, -1], [-1, -1]])  # матрица 2х2 с отрицательными элементами
        self.assertEqual(calculate_diagonal_sum(matrix), 0)  # Проверяем ответ

    def test_matrix(self):  # Тестирование функции на матрице с различными элементами
        matrix = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])  # 3x3 матрица
        self.assertEqual(calculate_diagonal_sum(matrix), 15)  # Проверяем ответ


if __name__ == '__main__':  # Для запуска тестов
    unittest.main()  # запуск тестов
