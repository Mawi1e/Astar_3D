# **Astar_3D**

+ A project that performs the **Astar** algorithm in a 3D map.
+ 3Dマップで **Astar**アルゴリズムを実行するプロジェクトです。
+ 3D맵에서 **Astar**알고리즘을 수행하는 프로젝트입니다.

### [--Astar_2D--](https://github.com/Mawi1e/Astar_2D)

## **mawile::LanguageSelector**

Lets you choose **the language that is output.**
**出力される言語**を選ぶことができます。
**출력되는 언어**를 고를 수 있습니다.

```cpp
enum class LanguageSelector {
		English = 1,
		Japanese = 2,
		Korean = 3,
};
```

## **Astar::Astar(bool)**

You can prevent **diagonal movements.**
**斜め向かいの移動**を防止できます。
**대각선 이동**을 방지할 수 있습니다.

```cpp
explicit(true) Astar(bool) noexcept;
```

# **Map File Format**

<br></br>
> **Units**
```
0: empty space, 空きスペース, 빈 공간
1: map, マップ, 맵
2: Departure point, 出発地点, 출발지점
3: Arrival point, 到着地点, 도착지점
4: Shortest path searched, 探索された経路, 탐색된 경로
```

<br></br>
> **Format**
```
[Col] [Row] [Depth]
[Map Information]

[横の長さ] [縦の長さ] [深さ]
[マップ情報]

[가로길이] [세로길이] [깊이]
[맵정보]
```

<br></br>
> **Examples**
```
3 3 3

0 0 0
0 2 0
0 0 0

0 1 1
1 1 1
1 1 1

0 0 0
0 3 0
0 0 0
```


# **Pictures**

![](https://github.com/Mawi1e/Astar_3D/blob/main/Pictures/1_1.PNG)
![](https://github.com/Mawi1e/Astar_3D/blob/main/Pictures/1_2.PNG)
![](https://github.com/Mawi1e/Astar_3D/blob/main/Pictures/2_1.PNG)
![](https://github.com/Mawi1e/Astar_3D/blob/main/Pictures/2_2.PNG)


# **Reference**

* **[GeeksforGeeks](https://www.geeksforgeeks.org/)**
