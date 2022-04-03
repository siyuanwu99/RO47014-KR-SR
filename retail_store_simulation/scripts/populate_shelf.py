
shelf_width = 1.0
shelf_depth = 0.5

def populate_board(product, height, amount, aruco_start=None):
    products = ""
    for i in range(amount):
        print(i, shelf_width, amount)
        if aruco_start is not None:
            products += \
            """
    <include>
      <uri>model://{}_aruco_{}</uri>
      <pose>{} 0 0 0 0 1.57</pose>
    </include>
            """.format(product, aruco_start+i, (i+1)*(shelf_width/(amount+1)))
        else:
            products += """
    <include>
      <uri>model://{}</uri>
      <pose>{} 0 0 0 0 1.57</pose>
    </include>
            """.format(product, (i+1)*(shelf_width/(amount+1)))

    return """
  <model name="board">
    <pose>0 0.5 {} 0 0 0</pose>
    {}
  </model>
    """.format(height, products)



def main():
    shelf_board_configurations = [
        # name, height, amount, ?aruco_start_id
        ["AH_hagelslag",        0.25, 8, 10],
        ["AH_hagelslag",        0.65, 8, 0],
        ["AH_thee_zwart_mango", 0.90, 6, 200],
        ["AH_hagelslag",        1.20, 8, 10],
        ["AH_hagelslag"       , 1.60, 6, 0],
        ["AH_thee_zwart_mango", 2.10, 6, 200],
    ]

    boards = ""
    for conf in shelf_board_configurations:
        boards += populate_board(*conf)

    template = \
"""
<model name="shelf">
  <include>
    <uri>model://AH_shelf</uri>
  </include>
  <pose>0 0 0 0 0 0</pose>

  {}
</model>
""".format(boards)

    print(template)


if __name__ == "__main__":
    main()
